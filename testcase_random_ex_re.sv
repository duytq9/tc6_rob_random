//******************************************************************************
//  rob_seq_random_constrained.sv  (rev 5)
//
//  Testcase UVM RANDOM CÓ RÀNG BUỘC cho ROB BOOM core
//  ────────────────────────────────────────────────────────────────────────────
//
//  Vị trí chèn:  bên trong package rob_pkg ở rob_uvm_tb_v4.sv
//
//  Thay đổi rev 5 so với rev 4:
//   - LÀM TƯỜNG MINH RULE: "Clean redispatch phải đi qua ĐÚNG cổng dispatch
//     (enq_X) đã gây exception."
//     • Rename field bank → PORT trong cả 3 struct cho khớp ngữ nghĩa
//       (port = kênh dispatch enq[X], khác với "ROB bank" trong DUT).
//     • Thêm `assert` runtime: trong do_clean_redispatch chỉ duy nhất
//       enq_valids[port] = 1, các port khác = 0.
//     • Log rõ ràng "enq_<port> rob_idx=<X>" trong cả lúc bắt exception
//       và lúc clean redispatch.
//
//  ============================================================================
//  TỔNG HỢP RULES (mọi version)
//  ============================================================================
//
//   1. Dispatch & WB tuân theo rob_tail_idx / rob_head_idx của DUT.
//      - Khi vif.ready = 0  → KHÔNG dispatch.
//      - WB chỉ gửi cho rob_idx + pdst đã thực sự dispatch.
//      - Tham số dispatch random TRỪ rob_idx và ftq_idx tuân theo thứ tự.
//
//   2. rob_idx của 1 row dispatch theo tail liên tục, ftq_idx CHUNG cả row.
//
//   3. Branch tag (br_mask) cấp phát từ bit thấp đến cao.
//      Per-bank: bank ≤ br_bank → mask_old; bank > br_bank → mask_new.
//      Lệnh branch chính nó: is_br=1, br_mask = mask_old.
//
//   4. mispredict() / resolve_correct() chỉ gọi với bit đã allocate.
//      Mispredict truyền rob_idx = tag_owner của bit.
//
//   5. Branch đã WB (tag_wb_done=1) → KHÔNG được mispredict bit đó nữa.
//
//   6. *** RULE QUAN TRỌNG (rev 5) ***
//      Khi 1 lệnh dispatch qua cổng enq_X được mark exception và sau đó
//      gây flush, redispatch sạch BẮT BUỘC qua đúng cổng enq_X đã dùng,
//      với rob_idx không đổi và exception=0.
//      → Tracking lưu PORT (không phải bank ROB) tại lúc dispatch.
//
//   7. Fence (is_fence=1, is_unique=1, unsafe=0) dispatch alone tại enq_0,
//      không track WB (busy=0 ngay). DUT vào wait_till_empty → ready=0
//      → seq tự idle đến khi DUT ready trở lại.
//
//******************************************************************************

  // ============================================================================
  //  rand_constr_seq
  // ============================================================================
  class rand_constr_seq extends rob_seq_base;
    `uvm_object_utils(rand_constr_seq)

    rob_driver drv;

    int unsigned NUM_CYCLES = 800;

    // -----------------------------------------------------------------------
    //  Live entry tracking
    //
    //  port = chỉ số cổng dispatch enq[port] tại lúc entry này được tạo.
    //         Không nhầm với "ROB bank" (= rob_idx % CW).
    //         Trong test này lane index trong transaction == port enq.
    // -----------------------------------------------------------------------
    typedef struct {
      bit [6:0]  rob_idx;
      bit [6:0]  pdst;
      bit [19:0] br_mask;
      bit        is_br;
      int        br_tag_bit;
      bit        has_exception;
      int        port;             // <<< RENAMED from bank
    } live_entry_t;

    live_entry_t live_q[$];

    // -----------------------------------------------------------------------
    //  Branch tag pool
    // -----------------------------------------------------------------------
    bit        tag_alive   [20];
    bit [6:0]  tag_owner   [20];
    bit        tag_wb_done [20];
    bit [19:0] current_br_mask;

    // -----------------------------------------------------------------------
    //  Exception tracking — parallel queue, không xoá khi WB
    // -----------------------------------------------------------------------
    typedef struct {
      bit [6:0]  rob_idx;
      int        port;             // <<< RENAMED
      bit [19:0] br_mask;
    } excepted_entry_t;

    excepted_entry_t excepted_q[$];

    // -----------------------------------------------------------------------
    //  Pending clean redispatch
    // -----------------------------------------------------------------------
    typedef struct {
      bit [6:0] rob_idx;
      int       port;              // <<< RENAMED — port BẮT BUỘC khi redispatch
    } pending_clean_t;

    bit             pending_clean_valid;
    pending_clean_t pending_clean;

    // -----------------------------------------------------------------------
    //  Misc
    // -----------------------------------------------------------------------
    int ftq_counter;
    int cycle_since_last_dispatch;
    int dispatch_count, wb_count, resolve_count, mis_count, idle_count;
    int fence_count, exception_count, redispatch_count;

    // -----------------------------------------------------------------------
    function new(string name = "rand_constr_seq");
      super.new(name);
    endfunction

    // ========================================================================
    //  body
    // ========================================================================
    task body();
      int action;

      get_vif();
      begin
        uvm_component comp = uvm_top.find("*.agt.drv");
        if (!$cast(drv, comp)) `uvm_fatal("RAND", "Driver cast failed")
      end

      foreach (tag_alive[i])   tag_alive[i]   = 1'b0;
      foreach (tag_owner[i])   tag_owner[i]   = 7'h0;
      foreach (tag_wb_done[i]) tag_wb_done[i] = 1'b0;
      current_br_mask           = 20'h0;
      ftq_counter               = 0;
      cycle_since_last_dispatch = 1;
      pending_clean_valid       = 1'b0;
      dispatch_count = 0; wb_count = 0; resolve_count = 0;
      mis_count = 0; idle_count = 0;
      fence_count = 0; exception_count = 0; redispatch_count = 0;

      `uvm_info("RAND",
        $sformatf("============== RANDOM ROB SEQ : %0d cycles ==============",
        NUM_CYCLES), UVM_LOW)

      repeat (3) @(posedge vif.clock);

      for (int cyc = 0; cyc < NUM_CYCLES; cyc++) begin
        @(posedge vif.clock);
        update_after_dut_events();
        action = pick_action();
        case (action)
          0: do_dispatch();
          1: do_wb();
          2: do_resolve();
          3: do_mispredict();
          default: do_idle();
        endcase
      end

      `uvm_info("RAND", "--- Drain phase ---", UVM_LOW)
      drain();

      `uvm_info("RAND",
        $sformatf("Stats: D=%0d W=%0d R=%0d M=%0d I=%0d  Fence=%0d Excp=%0d Redisp=%0d",
          dispatch_count, wb_count, resolve_count, mis_count, idle_count,
          fence_count, exception_count, redispatch_count), UVM_LOW)
      `uvm_info("RAND", "============== RANDOM SEQ COMPLETE ==============", UVM_LOW)
    endtask

    // ========================================================================
    //  pick_action
    // ========================================================================
    function int pick_action();
      int wd, ww, wr, wm, wi;
      int n_alive_br, n_mis_eligible;
      int total, r;

      // pending_clean → ưu tiên tuyệt đối
      if (pending_clean_valid) begin
        if (vif.mon_cb.ready) return 0;
        return 4;
      end

      n_alive_br     = count_alive_tags();
      n_mis_eligible = count_mispredict_eligible();

      wd = 40; ww = 30; wr = 8; wm = 5; wi = 17;

      if (!vif.mon_cb.ready) begin
        wd = 0; ww = 50; wm = 0; wr = 0; wi = 50;
      end

      if (live_q.size() == 0) begin
        ww = 0; wm = 0; wr = 0;
        if (vif.mon_cb.ready) begin wd = 70; wi = 30; end
        else                  begin wd = 0;  wi = 100; end
      end

      if (vif.mon_cb.empty) begin ww = 0; wm = 0; wr = 0; end

      if (vif.mon_cb.commit_rollback || vif.mon_cb.flush_valid) begin
        wd = 0; ww = 0; wm = 0; wr = 0; wi = 100;
      end

      if (cycle_since_last_dispatch == 0) ww = 0;
      if (n_alive_br == 0)     wr = 0;
      if (n_mis_eligible == 0) wm = 0;

      if (n_alive_br >= 18) begin
        wd = 0;
        wr = wr + 30;
        wm = wm + ((n_mis_eligible > 0) ? 20 : 0);
      end

      total = wd + ww + wr + wm + wi;
      if (total == 0) return 4;
      r = $urandom_range(0, total - 1);

      if (r < wd)                  return 0;
      if (r < wd + ww)             return 1;
      if (r < wd + ww + wr)        return 2;
      if (r < wd + ww + wr + wm)   return 3;
                                   return 4;
    endfunction

    // ========================================================================
    //  do_dispatch
    //   Ghi nhận ports (kênh enq) cho mọi entry — đặc biệt critical với
    //   exception entries (cần port để clean redispatch sau flush).
    // ========================================================================
    task do_dispatch();
      rob_transaction tr;
      bit [6:0]  cur_tail;
      int        num_valid;
      int        row_kind;
      int        special_port;     // port của branch / exception trong row
      int        new_bit;
      bit [19:0] mask_old, mask_new;
      int        r;

      if (pending_clean_valid) begin
        do_clean_redispatch();
        return;
      end

      if (!vif.mon_cb.ready) begin
        do_idle();
        return;
      end

      cur_tail  = vif.mon_cb.rob_tail_idx;
      num_valid = $urandom_range(1, 4);

      r = $urandom_range(0, 99);
      if      (r < 60) row_kind = 0;
      else if (r < 80) row_kind = 1;
      else if (r < 88) row_kind = 2;
      else             row_kind = 3;

      if (row_kind == 2) num_valid = 1;

      special_port = 0;
      if (row_kind == 1 || row_kind == 3)
        special_port = $urandom_range(0, num_valid - 1);

      mask_old = current_br_mask;
      mask_new = mask_old;
      new_bit  = -1;

      if (row_kind == 1) begin
        new_bit = alloc_br_bit();
        if (new_bit < 0) begin
          row_kind = 0;
        end else begin
          mask_new = mask_old | (20'h1 << new_bit);
        end
      end

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                = ROB_DISPATCH;
      tr.enq_partial_stall = 1'b0;

      // Chú ý: chỉ số b trong vòng for này CHÍNH LÀ port enq[b]
      for (int b = 0; b < CW; b++) begin
        tr.enq_valids[b]              = (b < num_valid);
        tr.enq_uops_rob_idx[b]        = 7'(cur_tail + b);
        tr.enq_uops_ftq_idx[b]        = 6'(ftq_counter);
        tr.enq_uops_pc_lob[b]         = 6'(b * 4);

        tr.enq_uops_uopc[b]           = 7'($urandom_range(0, 127));
        tr.enq_uops_pdst[b]           = 7'($urandom_range(1, 127));
        tr.enq_uops_stale_pdst[b]     = 7'($urandom_range(0, 127));
        tr.enq_uops_ldst[b]           = 6'($urandom_range(1, 31));
        tr.enq_uops_ldst_val[b]       = 1'b1;
        tr.enq_uops_dst_rtype[b]      = 2'b01;
        tr.enq_uops_fp_val[b]         = 1'b0;
        tr.enq_uops_unsafe[b]         = !(row_kind == 2 && b == 0);

        if (row_kind == 1) begin
          tr.enq_uops_is_br[b]   = (b == special_port);
          tr.enq_uops_br_mask[b] = (b <= special_port) ? mask_old : mask_new;
        end else begin
          tr.enq_uops_is_br[b]   = 1'b0;
          tr.enq_uops_br_mask[b] = mask_old;
        end

        if (row_kind == 2 && b == 0) begin
          tr.enq_uops_is_fence[b]  = 1'b1;
          tr.enq_uops_is_fencei[b] = 1'b0;
          tr.enq_uops_is_unique[b] = 1'b1;
        end else begin
          tr.enq_uops_is_fence[b]  = 1'b0;
          tr.enq_uops_is_fencei[b] = 1'b0;
          tr.enq_uops_is_unique[b] = 1'b0;
        end

        if (row_kind == 3 && b == special_port) begin
          tr.enq_uops_exception[b] = 1'b1;
          tr.enq_uops_exc_cause[b] = 64'h2;
        end else begin
          tr.enq_uops_exception[b] = 1'b0;
          tr.enq_uops_exc_cause[b] = 64'd0;
        end

        tr.enq_uops_uses_ldq[b]        = 1'b0;
        tr.enq_uops_uses_stq[b]        = 1'b0;
        tr.enq_uops_flush_on_commit[b] = 1'b0;
      end
      finish_item(tr);
      @(posedge vif.clock);

      // ----------------------------------------------------------------------
      //  Update tracking — GHI NHẬN PORT cho mọi entry
      // ----------------------------------------------------------------------
      for (int b = 0; b < num_valid; b++) begin
        bit is_fence_b = (row_kind == 2 && b == 0);
        bit is_excp_b  = (row_kind == 3 && b == special_port);
        live_entry_t e;

        if (is_fence_b) continue;

        e.rob_idx       = 7'(cur_tail + b);
        e.pdst          = tr.enq_uops_pdst[b];
        e.br_mask       = tr.enq_uops_br_mask[b];
        e.is_br         = (row_kind == 1 && b == special_port);
        e.br_tag_bit    = (row_kind == 1 && b == special_port) ? new_bit : -1;
        e.has_exception = is_excp_b;
        e.port          = b;                  // <<< Lưu port enq[b]
        live_q.push_back(e);

        if (is_excp_b) begin
          excepted_entry_t exc;
          exc.rob_idx = e.rob_idx;
          exc.port    = b;                    // <<< Lưu port enq[b]
          exc.br_mask = e.br_mask;
          excepted_q.push_back(exc);

          `uvm_info("RAND",
            $sformatf("[D-EXCP] enq_%0d rob_idx=%0d  → excepted_q (sẽ phải redispatch lại đúng kênh enq_%0d)",
              b, e.rob_idx, b), UVM_LOW)
        end
      end

      if (row_kind == 1 && new_bit >= 0) begin
        tag_alive  [new_bit] = 1'b1;
        tag_owner  [new_bit] = 7'(cur_tail + special_port);
        tag_wb_done[new_bit] = 1'b0;
        current_br_mask      = mask_new;
      end

      ftq_counter               = (ftq_counter + 1) & 6'h3F;
      cycle_since_last_dispatch = 0;
      dispatch_count++;
      if (row_kind == 2) fence_count++;
      if (row_kind == 3) exception_count++;

      `uvm_info("RAND",
        $sformatf("[D] tail=%0d nv=%0d kind=%0d sp_port=enq_%0d new_bit=%0d mask=%05h",
          cur_tail, num_valid, row_kind, special_port, new_bit, current_br_mask),
        UVM_HIGH)
    endtask

    // ========================================================================
    //  do_clean_redispatch — REDISPATCH SẠCH SAU FLUSH-DO-EXCEPTION
    //
    //   RULE BẮT BUỘC: enq_valids[pending_clean.port] = 1 và các port khác = 0
    //                  enq_uops_rob_idx[pending_clean.port] = pending_clean.rob_idx
    //                  enq_uops_exception[pending_clean.port] = 0
    //   → đảm bảo lệnh sạch quay lại đúng kênh enq đã từng gây exception.
    // ========================================================================
    task do_clean_redispatch();
      rob_transaction tr;
      bit [6:0]       cur_tail;
      int             cnt_valids;

      if (!vif.mon_cb.ready) begin
        do_idle();
        return;
      end

      cur_tail = vif.mon_cb.rob_tail_idx;

      // Tail phải đã rollback đúng vị trí. Nếu chưa → idle chờ.
      if (cur_tail !== pending_clean.rob_idx) begin
        `uvm_info("RAND",
          $sformatf("[CR] wait rollback: vif.tail=%0d expected=%0d",
            cur_tail, pending_clean.rob_idx), UVM_HIGH)
        do_idle();
        return;
      end

      // Sanity: port phải trong [0, CW-1]
      if (pending_clean.port < 0 || pending_clean.port >= CW)
        `uvm_fatal("RAND",
          $sformatf("[CR] pending_clean.port=%0d ngoài [0,%0d)",
            pending_clean.port, CW))

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                = ROB_DISPATCH;
      tr.enq_partial_stall = 1'b0;

      cnt_valids = 0;
      for (int b = 0; b < CW; b++) begin
        // CHỈ port pending_clean.port được valid — RULE
        bit is_target = (b == pending_clean.port);

        tr.enq_valids[b]          = is_target;
        tr.enq_uops_rob_idx[b]    = is_target ? pending_clean.rob_idx : 7'h0;
        tr.enq_uops_ftq_idx[b]    = 6'(ftq_counter);
        tr.enq_uops_pc_lob[b]     = 6'(b * 4);

        tr.enq_uops_uopc[b]       = 7'($urandom_range(0, 127));
        tr.enq_uops_pdst[b]       = 7'($urandom_range(1, 127));
        tr.enq_uops_stale_pdst[b] = 7'($urandom_range(0, 127));
        tr.enq_uops_ldst[b]       = 6'($urandom_range(1, 31));
        tr.enq_uops_ldst_val[b]   = 1'b1;
        tr.enq_uops_dst_rtype[b]  = 2'b01;
        tr.enq_uops_fp_val[b]     = 1'b0;
        tr.enq_uops_unsafe[b]     = 1'b1;

        // Sạch tuyệt đối
        tr.enq_uops_is_br[b]           = 1'b0;
        tr.enq_uops_br_mask[b]         = current_br_mask;
        tr.enq_uops_is_fence[b]        = 1'b0;
        tr.enq_uops_is_fencei[b]       = 1'b0;
        tr.enq_uops_is_unique[b]       = 1'b0;
        tr.enq_uops_exception[b]       = 1'b0;       // <<< KEY: clean
        tr.enq_uops_exc_cause[b]       = 64'd0;
        tr.enq_uops_uses_ldq[b]        = 1'b0;
        tr.enq_uops_uses_stq[b]        = 1'b0;
        tr.enq_uops_flush_on_commit[b] = 1'b0;

        if (tr.enq_valids[b]) cnt_valids++;
      end

      // ASSERT: chỉ duy nhất 1 port valid và đó phải là pending_clean.port
      if (cnt_valids !== 1)
        `uvm_fatal("RAND",
          $sformatf("[CR] Sai số lượng valid: %0d (phải = 1)", cnt_valids))
      if (!tr.enq_valids[pending_clean.port])
        `uvm_fatal("RAND",
          $sformatf("[CR] Sai port: enq_%0d không được set valid",
            pending_clean.port))
      if (tr.enq_uops_rob_idx[pending_clean.port] !== pending_clean.rob_idx)
        `uvm_fatal("RAND",
          $sformatf("[CR] Sai rob_idx: enq_%0d=%0d, expected=%0d",
            pending_clean.port,
            tr.enq_uops_rob_idx[pending_clean.port],
            pending_clean.rob_idx))
      if (tr.enq_uops_exception[pending_clean.port] !== 1'b0)
        `uvm_fatal("RAND",
          $sformatf("[CR] enq_%0d.exception phải = 0", pending_clean.port))

      finish_item(tr);
      @(posedge vif.clock);

      // Add vào live_q với chính port đó để có thể được WB
      begin
        live_entry_t e;
        e.rob_idx       = pending_clean.rob_idx;
        e.pdst          = tr.enq_uops_pdst[pending_clean.port];
        e.br_mask       = current_br_mask;
        e.is_br         = 1'b0;
        e.br_tag_bit    = -1;
        e.has_exception = 1'b0;
        e.port          = pending_clean.port;       // <<< giữ port
        live_q.push_back(e);
      end

      ftq_counter               = (ftq_counter + 1) & 6'h3F;
      cycle_since_last_dispatch = 0;
      dispatch_count++;
      redispatch_count++;

      `uvm_info("RAND",
        $sformatf("[CR] CLEAN redispatch  enq_%0d  rob_idx=%0d  → live_q (clean, có thể WB)",
          pending_clean.port, pending_clean.rob_idx), UVM_LOW)

      pending_clean_valid = 1'b0;
    endtask

    // ========================================================================
    //  do_wb
    // ========================================================================
    task do_wb();
      rob_transaction tr;
      int n_wb;

      if (live_q.size() == 0) begin do_idle(); return; end

      n_wb = $urandom_range(1, (live_q.size() < 10) ? live_q.size() : 10);

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op = ROB_WRITEBACK;
      for (int p = 0; p < NUM_WB_PORTS; p++) tr.wb_valid[p] = 1'b0;

      for (int i = 0; i < n_wb; i++) begin
        int          pick;
        live_entry_t le;

        if (live_q.size() == 0) break;
        pick = $urandom_range(0, live_q.size() - 1);
        le   = live_q[pick];

        if (drv.can_writeback(le.rob_idx)) begin
          tr.wb_valid[i]   = 1'b1;
          tr.wb_rob_idx[i] = le.rob_idx;
          tr.wb_pdst[i]    = le.pdst;

          if (le.is_br && le.br_tag_bit >= 0
              && tag_alive[le.br_tag_bit]
              && tag_owner[le.br_tag_bit] == le.rob_idx) begin
            tag_wb_done[le.br_tag_bit] = 1'b1;
            `uvm_info("RAND",
              $sformatf("[W] branch rob_idx=%0d (bit %0d) WB-done",
                le.rob_idx, le.br_tag_bit), UVM_MEDIUM)
          end

          if (le.has_exception)
            `uvm_info("RAND",
              $sformatf("[W] EXCEPTION enq_%0d rob_idx=%0d WB-done — chờ commit→flush",
                le.port, le.rob_idx), UVM_MEDIUM)

          wb_count++;
          `uvm_info("RAND",
            $sformatf("[W] rob_idx=%0d pdst=%0d", le.rob_idx, le.pdst), UVM_HIGH)
        end

        live_q.delete(pick);
      end

      finish_item(tr);
      @(posedge vif.clock);
      cycle_since_last_dispatch++;
    endtask

    // ========================================================================
    //  do_resolve
    // ========================================================================
    task do_resolve();
      rob_transaction tr;
      int             alive_bits[$];
      int             pick, bit_pos;
      bit [19:0]      mask;

      get_alive_bits(alive_bits);
      if (alive_bits.size() == 0) begin do_idle(); return; end

      pick    = $urandom_range(0, alive_bits.size() - 1);
      bit_pos = alive_bits[pick];
      mask    = 20'h1 << bit_pos;

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = mask;
      tr.br_mispredict_mask = 20'h0;
      tr.br_rob_idx         = 7'h0;
      tr.br_mispredict      = 1'b0;
      tr.br_taken           = 1'b0;
      finish_item(tr);
      @(posedge vif.clock);

      foreach (live_q[i]) begin
        live_q[i].br_mask &= ~mask;
        if (live_q[i].is_br && live_q[i].br_tag_bit == bit_pos) begin
          live_q[i].is_br      = 1'b0;
          live_q[i].br_tag_bit = -1;
        end
      end
      foreach (excepted_q[i]) excepted_q[i].br_mask &= ~mask;

      tag_alive  [bit_pos] = 1'b0;
      tag_wb_done[bit_pos] = 1'b0;
      current_br_mask    &= ~mask;

      cycle_since_last_dispatch++;
      resolve_count++;
      `uvm_info("RAND",
        $sformatf("[R] resolve_correct mask=%05h", mask), UVM_MEDIUM)
    endtask

    // ========================================================================
    //  do_mispredict
    // ========================================================================
    task do_mispredict();
      rob_transaction tr;
      int             eligible_bits[$];
      int             pick, bit_pos;
      bit [19:0]      mask;
      bit [6:0]       br_idx;

      get_mispredict_eligible(eligible_bits);
      if (eligible_bits.size() == 0) begin do_idle(); return; end

      pick    = $urandom_range(0, eligible_bits.size() - 1);
      bit_pos = eligible_bits[pick];
      mask    = 20'h1 << bit_pos;
      br_idx  = tag_owner[bit_pos];

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = mask;
      tr.br_mispredict_mask = mask;
      tr.br_rob_idx         = br_idx;
      tr.br_mispredict      = 1'b1;
      tr.br_taken           = $urandom_range(0, 1);
      finish_item(tr);
      @(posedge vif.clock);

      for (int i = live_q.size() - 1; i >= 0; i--) begin
        if (live_q[i].br_mask & mask) begin
          if (live_q[i].is_br && live_q[i].br_tag_bit >= 0) begin
            int other_bit = live_q[i].br_tag_bit;
            if (tag_alive[other_bit] &&
                tag_owner[other_bit] == live_q[i].rob_idx) begin
              tag_alive  [other_bit] = 1'b0;
              tag_wb_done[other_bit] = 1'b0;
              current_br_mask      &= ~(20'h1 << other_bit);
            end
          end
          live_q.delete(i);
        end
      end

      // Kill excepted_q entries phụ thuộc nhánh bị mispredict
      for (int i = excepted_q.size() - 1; i >= 0; i--) begin
        if (excepted_q[i].br_mask & mask) begin
          `uvm_info("RAND",
            $sformatf("[M] kill excepted enq_%0d rob_idx=%0d (br_mask=%05h)",
              excepted_q[i].port, excepted_q[i].rob_idx, excepted_q[i].br_mask),
            UVM_MEDIUM)
          excepted_q.delete(i);
        end
      end

      tag_alive  [bit_pos] = 1'b0;
      tag_wb_done[bit_pos] = 1'b0;
      current_br_mask    &= ~mask;

      foreach (live_q[i]) begin
        if (live_q[i].is_br && live_q[i].br_tag_bit == bit_pos) begin
          live_q[i].is_br      = 1'b0;
          live_q[i].br_tag_bit = -1;
        end
      end

      mis_count++;
      `uvm_info("RAND",
        $sformatf("[M] mispredict bit=%0d br_idx=%0d → live=%0d excp=%0d",
          bit_pos, br_idx, live_q.size(), excepted_q.size()), UVM_MEDIUM)

      cycle_since_last_dispatch++;
      repeat ($urandom_range(3, 8)) do_idle();
    endtask

    // ========================================================================
    //  do_idle
    // ========================================================================
    task do_idle();
      rob_transaction tr;
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op = ROB_IDLE;
      finish_item(tr);
      @(posedge vif.clock);
      cycle_since_last_dispatch++;
      idle_count++;
    endtask

    // ========================================================================
    //  update_after_dut_events
    //   flush_valid → set pending_clean = oldest excepted (rob_idx + port)
    // ========================================================================
    function void update_after_dut_events();
      if (vif.mon_cb.flush_valid) begin
        pending_clean_valid = 1'b0;
        if (excepted_q.size() > 0) begin
          pending_clean.rob_idx = excepted_q[0].rob_idx;
          pending_clean.port    = excepted_q[0].port;     // <<< giữ port
          pending_clean_valid   = 1'b1;
          `uvm_info("RAND",
            $sformatf("[E] flush_valid → PENDING CLEAN: enq_%0d rob_idx=%0d (phải redispatch sạch đúng kênh này)",
              pending_clean.port, pending_clean.rob_idx), UVM_LOW)
        end else begin
          `uvm_info("RAND",
            "[E] flush_valid (no excepted entry tracked) → clear", UVM_MEDIUM)
        end

        live_q.delete();
        excepted_q.delete();
        foreach (tag_alive[i])   tag_alive[i]   = 1'b0;
        foreach (tag_wb_done[i]) tag_wb_done[i] = 1'b0;
        current_br_mask = 20'h0;
      end
    endfunction

    // ========================================================================
    //  drain
    // ========================================================================
    task drain();
      int attempts = 0;

      while (attempts < 200) begin
        @(posedge vif.clock);
        update_after_dut_events();

        if (pending_clean_valid) begin
          do_clean_redispatch();
          attempts++;
          continue;
        end

        if (live_q.size() > 0 && cycle_since_last_dispatch > 0) begin
          do_wb();
        end
        else if (count_alive_tags() > 0) begin
          do_resolve();
        end
        else if (live_q.size() == 0 && excepted_q.size() == 0
                 && !pending_clean_valid && vif.mon_cb.empty) begin
          `uvm_info("RAND", "Drain complete: ROB empty", UVM_LOW)
          return;
        end
        else begin
          do_idle();
        end

        attempts++;
      end

      if (!vif.mon_cb.empty)
        `uvm_warning("RAND",
          $sformatf("Drain timeout: live=%0d excp=%0d pending=%0b empty=%0b",
            live_q.size(), excepted_q.size(), pending_clean_valid,
            vif.mon_cb.empty))
    endtask

    // ========================================================================
    //  Helpers
    // ========================================================================
    function int alloc_br_bit();
      for (int i = 0; i < 20; i++) if (!tag_alive[i]) return i;
      return -1;
    endfunction

    function int count_alive_tags();
      int cnt = 0;
      for (int i = 0; i < 20; i++) if (tag_alive[i]) cnt++;
      return cnt;
    endfunction

    function int count_mispredict_eligible();
      int cnt = 0;
      for (int i = 0; i < 20; i++)
        if (tag_alive[i] && !tag_wb_done[i]) cnt++;
      return cnt;
    endfunction

    function void get_alive_bits(ref int bits_q[$]);
      bits_q.delete();
      for (int i = 0; i < 20; i++) if (tag_alive[i]) bits_q.push_back(i);
    endfunction

    function void get_mispredict_eligible(ref int bits_q[$]);
      bits_q.delete();
      for (int i = 0; i < 20; i++)
        if (tag_alive[i] && !tag_wb_done[i]) bits_q.push_back(i);
    endfunction

  endclass : rand_constr_seq


  // ============================================================================
  //  Test class
  //  Lệnh chạy:  +UVM_TESTNAME=rand_constr_test [+NUM_CYCLES=2000]
  // ============================================================================
  class rand_constr_test extends rob_base_test;
    `uvm_component_utils(rand_constr_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      rand_constr_seq seq;
      phase.raise_objection(this);
      seq = rand_constr_seq::type_id::create("seq");
      begin
        int n;
        if ($value$plusargs("NUM_CYCLES=%d", n)) seq.NUM_CYCLES = n;
      end
      seq.start(env.agt.sqr);
      phase.drop_objection(this);
    endtask
  endclass : rand_constr_test
