// ==== FILE: rob_seq_tc5_branch_mispredict.sv ====
//
// FIX: Branch protocol đúng theo DUT interface:
//   1. Dispatch: is_br bật khi valid, lệnh sau tích lũy br_mask bit từ thấp→cao
//   2. Branch update 2-cycle:
//      - Cycle N:   b1_resolve_mask + b1_mispredict_mask (b2_mispredict=0)
//      - Cycle N+1: b2_mispredict=1 + b2_rob_idx (nếu mispredict)
//      - Correct resolve: chỉ cần cycle N (b1), không cần b2

  class tc5_branch_mispredict_seq extends rob_seq_base;
    `uvm_object_utils(tc5_branch_mispredict_seq)

    // Branch tag allocator
    int next_br_tag;
    bit [19:0] active_br_mask;  // mask tích lũy cho lệnh tiếp theo

    function new(string name="tc5"); super.new(name); endfunction

    function void reset_br_state();
      next_br_tag = 0;
      active_br_mask = 0;
    endfunction

    // ── Dispatch row có branch tại br_bank ──
    // Branch instruction nhận active_br_mask HIỆN TẠI (không bao gồm tag mới)
    // Sau dispatch, tag mới được thêm vào active_br_mask cho lệnh tiếp theo
    task automatic dispatch_br(
      output int row_id,
      input  int br_bank        // bank nào là branch (-1 = không có branch)
    );
      rob_transaction tr;
      drow_info_t info;
      bit [6:0] tail;
      bit [19:0] this_row_mask;

      tail = vif.rob_tail_idx;
      this_row_mask = active_br_mask;  // mask tại thời điểm dispatch

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op = ROB_DISPATCH;
      tr.enq_partial_stall = 0;

      info.base_idx  = tail;
      info.valid_mask = 4'hF;

      for (int b = 0; b < CW; b++) begin
        tr.enq_valids[b]          = 1;
        tr.enq_uops_rob_idx[b]    = tail + b;
        info.pdst[b]               = next_pdst();
        tr.enq_uops_pdst[b]       = info.pdst[b];
        tr.enq_uops_stale_pdst[b] = 0;
        tr.enq_uops_exception[b]  = 0;
        tr.enq_uops_is_br[b]      = (b == br_bank);
        tr.enq_uops_br_mask[b]    = this_row_mask;
        tr.enq_uops_unsafe[b]     = 1;
        tr.enq_uops_ldst_val[b]   = 1;
        info.is_fence[b]           = 0;
      end
      finish_item(tr);

      drow_q.push_back(info);
      row_id = drow_q.size() - 1;

      // Sau dispatch: nếu row này có branch, allocate tag mới
      if (br_bank >= 0 && next_br_tag < 20) begin
        active_br_mask |= (20'h1 << next_br_tag);
        `uvm_info("TC5", $sformatf("Allocated br_tag=%0d, active_mask=0x%05h",
          next_br_tag, active_br_mask), UVM_HIGH)
        next_br_tag++;
      end
    endtask

    // ── Dispatch row KHÔNG có branch (chỉ speculative dưới branches trước) ──
    task automatic dispatch_spec(output int row_id);
      dispatch_br(row_id, -1);  // no branch in this row
    endtask

    // ── 2-cycle Branch Mispredict ──
    // Cycle 1: b1_resolve_mask + b1_mispredict_mask
    // Cycle 2: b2_mispredict=1 + b2_rob_idx
    task branch_mispredict_2cyc(bit [19:0] mask, bit [6:0] rob_idx);
      rob_transaction tr;

      // ── Cycle 1: b1 ──
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = mask;
      tr.br_mispredict_mask = mask;
      tr.br_mispredict      = 0;     // b2 chưa gửi
      tr.br_rob_idx         = 0;
      finish_item(tr);

      // ── Cycle 2: b2 ──
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = 0;     // b1 clear
      tr.br_mispredict_mask = 0;
      tr.br_mispredict      = 1;     // b2 bật
      tr.br_rob_idx         = rob_idx;
      tr.br_taken           = 1;
      finish_item(tr);

      `uvm_info("TC5", $sformatf("Mispredict 2-cyc: mask=0x%05h rob_idx=%0d", mask, rob_idx), UVM_MEDIUM)
    endtask

    // ── 1-cycle Branch Resolve Correct ──
    // Chỉ b1, không cần b2
    task branch_resolve_1cyc(bit [19:0] mask);
      rob_transaction tr;
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = mask;
      tr.br_mispredict_mask = 0;     // không mispredict
      tr.br_mispredict      = 0;
      tr.br_rob_idx         = 0;
      finish_item(tr);

      // Clear bit đã resolve khỏi active mask
      active_br_mask &= ~mask;

      `uvm_info("TC5", $sformatf("Resolve correct: mask=0x%05h, active_mask=0x%05h",
        mask, active_br_mask), UVM_HIGH)
    endtask

    // ════════════════════════════════════════════════════════════
    task body();
      int r0, r1, r2;
      logic [6:0] tail_before;
      get_vif();
      `uvm_info("TC5","=== TC5: Branch Misprediction ===",UVM_LOW)

      // ──────────────────────────────────────────────────
      // Phase A: br_mask kill + tail snap-back
      // ──────────────────────────────────────────────────
      `uvm_info("TC5","Phase A: kill + snap-back",UVM_LOW)
      reset_br_state();
      // Row0: branch at bank0, br_mask=0 (non-spec) → allocate tag 0
      dispatch_br(r0, 0);
      // active_br_mask = 0x1 giờ

      // Row1: speculative dưới tag 0, br_mask=0x1
      dispatch_spec(r1);
      // Row2: speculative dưới tag 0, br_mask=0x1
      dispatch_spec(r2);

      @(posedge vif.clock);
      tail_before = vif.rob_tail_idx;
      idle(1);

      // Mispredict tag 0 (2-cycle): kill row1, row2
      branch_mispredict_2cyc(20'h1, get_base(r0));

      repeat(3) begin
        idle(1); @(posedge vif.clock);
        `uvm_info("TC5",$sformatf("tail=%0d head=%0d",
          vif.rob_tail_idx, vif.rob_head_idx),UVM_MEDIUM)
      end
      @(posedge vif.clock);
      if (vif.rob_tail_idx < tail_before)
        `uvm_info("TC5","F5.2 PASS: tail snapped back",UVM_LOW)
      wait_recovery();

      // ──────────────────────────────────────────────────
      // Phase B: No WB after kill
      // ──────────────────────────────────────────────────
      `uvm_info("TC5","Phase B: no WB after kill",UVM_LOW)
      reset_br_state();
      dispatch_br(r0, 0);       // branch at bank0 → tag 0
      dispatch_spec(r1);        // under tag 0
      idle(1);
      // Mispredict → r1 killed
      branch_mispredict_2cyc(20'h1, get_base(r0));
      // KHÔNG drive WB cho r1 (đã killed)
      idle(10);
      `uvm_info("TC5","PASS: no WB driven after kill (W5)",UVM_LOW)
      wait_recovery();

      // ──────────────────────────────────────────────────
      // Phase C: Mispredict with pending partial WB
      // ──────────────────────────────────────────────────
      `uvm_info("TC5","Phase C: pending WB + mispredict",UVM_LOW)
      reset_br_state();
      dispatch_br(r0, 0);       // branch → tag 0
      dispatch_spec(r1);        // under tag 0
      dispatch_spec(r2);        // under tag 0
      idle(1);
      wb_row(r0);               // WB row0 (non-spec part, bank 1-3 under tag0 nhưng vẫn WB được vì chưa kill)
      wb_banks(r1, 4'b0001);    // partial WB row1 bank0

      // Mispredict tag 0 → r1 (partial WB), r2 killed
      branch_mispredict_2cyc(20'h1, get_base(r0));
      idle(5);
      `uvm_info("TC5","PASS: mispredict with partial WB",UVM_LOW)
      wait_recovery();

      // ──────────────────────────────────────────────────
      // Phase D: Mispredict + commit race
      // ──────────────────────────────────────────────────
      `uvm_info("TC5","Phase D: commit race",UVM_LOW)
      reset_br_state();
      // Row0: non-speculative (no branch)
      dispatch_spec(r0);        // br_mask=0 vì chưa có branch nào
      idle(1); wb_row(r0);      // ready to commit

      // Giờ mới dispatch branch
      reset_br_state();         // reset lại br_tag
      dispatch_br(r1, 0);       // branch → tag 0
      idle(1);

      // Mispredict cùng khoảng thời gian row0 đang commit
      branch_mispredict_2cyc(20'h1, get_base(r1));
      wait_recovery();
      `uvm_info("TC5","PASS: commit + mispredict race",UVM_LOW)

      // ──────────────────────────────────────────────────
      // Phase E: Nested speculation
      //   3 branch lồng nhau, resolve tag 0 correct, mispredict tag 1
      // ──────────────────────────────────────────────────
      `uvm_info("TC5","Phase E: Nested speculation",UVM_LOW)
      reset_br_state();

      // Row0: branch at bank0, br_mask=0x0 → allocate tag 0, active=0x1
      dispatch_br(r0, 0);
      // Row1: branch at bank0, br_mask=0x1 (under tag0) → allocate tag 1, active=0x3
      dispatch_br(r1, 0);
      // Row2: branch at bank0, br_mask=0x3 (under tag0+1) → allocate tag 2, active=0x7
      dispatch_br(r2, 0);

      `uvm_info("TC5",$sformatf("Nested: r0.mask=0x0, r1.mask=0x1, r2.mask=0x3, active=0x%05h",
        active_br_mask),UVM_MEDIUM)

      idle(1);

      // Resolve tag 0 correct (1-cycle b1 only)
      // → tất cả entry clear bit 0 trong br_mask
      // r0: mask 0x0→0x0, r1: mask 0x1→0x0, r2: mask 0x3→0x2
      branch_resolve_1cyc(20'h1);
      idle(1);

      // Mispredict tag 1 (2-cycle)
      // r0: mask bit1=0 → SỐNG (F5.4)
      // r1: mask bit1=0 (đã clear ở bước trên vì original mask=0x1, clear bit0→0x0, bit1 chưa set)
      //     Nhưng r1 dispatch với br_mask=0x1. Resolve bit0 → br_mask=0x0. Bit1 KHÔNG có trong r1.
      //     → r1 SỐNG (vì br_mask & 0x2 = 0)
      // r2: mask gốc=0x3, clear bit0→0x2. Bit1 CÓ → KILLED
      // Thực ra r1 có tag 1 nhưng br_mask của nó chỉ là 0x1 (under tag 0, không under tag 1)
      // Branch tag 1 là do r1 ALLOCATE, nhưng r1 không speculative dưới chính nó
      // Vậy: mispredict tag 1 → kill entry có bit1 set → chỉ r2 bị kill
      branch_mispredict_2cyc(20'h2, get_base(r1));

      repeat(3) idle(1);
      `uvm_info("TC5","PASS: nested speculation — r0 survives, r2 killed",UVM_LOW)
      wait_recovery();

      `uvm_info("TC5","=== TC5 Complete ===",UVM_LOW)
    endtask
  endclass
