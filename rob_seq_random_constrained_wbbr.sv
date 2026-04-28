//******************************************************************************
//  rob_seq_random_constrained.sv  (rev 2)
//
//  Testcase UVM RANDOM CÓ RÀNG BUỘC cho ROB BOOM core
//  ────────────────────────────────────────────────────────────────────────────
//
//  Vị trí chèn:  bên trong package rob_pkg ở rob_uvm_tb_v4.sv
//                (ngay trước hoặc sau khối "rob_seq_tc6_random.sv")
//
//  Ràng buộc đã thực thi:
//   1. Dispatch & WB tuân theo rob_tail_idx / rob_head_idx của DUT.
//      - Khi vif.ready = 0  → KHÔNG dispatch.
//      - WB chỉ gửi cho rob_idx + pdst đã thực sự dispatch (qua live_q[$]).
//      - Tham số dispatch random TRỪ rob_idx và ftq_idx tuân theo thứ tự.
//
//   2. rob_idx của 1 row dispatch theo tail liên tục:
//      cur_tail, cur_tail+1, cur_tail+2, cur_tail+3.
//      Toàn bộ 4 bank trong 1 dispatch transaction CHUNG ftq_counter.
//      Mỗi dispatch transaction → ftq_counter++ (next row dùng ftq mới).
//
//   3. Cơ chế branch tag (br_mask):
//      - Khi gặp branch, allocate bit thấp nhất còn trống (bit 0 → 19).
//      - Lệnh BRANCH chính nó: is_br=1, br_mask = mask CŨ
//        (chưa mang bit của chính nó).
//      - Các lệnh SAU branch (cùng row, bank > br_bank) và mọi dispatch
//        sau đó: br_mask = mask MỚI (đã có bit mới).
//      Logic per-bank giống dispatch_branch_row trong rob_seq_helpers:
//          bank ≤ br_bank  → mask_old
//          bank >  br_bank → mask_new
//
//   4. mispredict() và resolve_correct() chỉ gọi với bit đã được allocate
//      (chỉ chọn từ pool tag_alive[]):
//      - resolve_correct(mask) → clear bit đó ở mọi entry, free tag.
//      - mispredict(rob_idx, mask) → kill entries phụ thuộc, free tag mispredict
//        và free tag của các branch trẻ hơn cũng bị kill (chuỗi dây chuyền).
//
//   5. *** RULE MỚI ***
//      Nếu lệnh BRANCH sở hữu 1 bit br_mark đã được WB rồi → KHÔNG được phát
//      mispredict trên bit đó nữa. Chỉ resolve_correct được phép.
//      → tracking thêm tag_wb_done[20]. do_mispredict chỉ pick bit còn:
//             tag_alive[i] = 1  AND  tag_wb_done[i] = 0
//
//******************************************************************************

  // ============================================================================
  //  rand_constr_seq — constrained-random sequence
  // ============================================================================
  class rand_constr_seq extends rob_seq_base;
    `uvm_object_utils(rand_constr_seq)

    // Driver handle (để gọi can_writeback shadow)
    rob_driver drv;

    // -----------------------------------------------------------------------
    //  Config
    // -----------------------------------------------------------------------
    int unsigned NUM_CYCLES = 800;

    // -----------------------------------------------------------------------
    //  Live entry tracking — entry đã dispatch còn alive
    // -----------------------------------------------------------------------
    typedef struct {
      bit [6:0]  rob_idx;
      bit [6:0]  pdst;
      bit [19:0] br_mask;       // br_mask tại thời điểm dispatch
      bit        is_br;
      int        br_tag_bit;    // -1 nếu không phải branch
    } live_entry_t;

    live_entry_t live_q[$];

    // -----------------------------------------------------------------------
    //  Branch tag pool — 20 bit tag được cấp phát từ thấp đến cao
    //
    //  3 trạng thái song song mỗi bit i (0..19):
    //    tag_alive[i]   = bit đang được allocated cho 1 lệnh branch
    //    tag_owner[i]   = rob_idx của lệnh branch đó
    //    tag_wb_done[i] = lệnh branch đã được WB rồi → cấm mispredict
    // -----------------------------------------------------------------------
    bit        tag_alive   [20];
    bit [6:0]  tag_owner   [20];
    bit        tag_wb_done [20];    //  <<< RULE MỚI
    bit [19:0] current_br_mask;     // = OR của tất cả tag_alive bit

    // -----------------------------------------------------------------------
    //  Misc tracking
    // -----------------------------------------------------------------------
    int ftq_counter;
    int cycle_since_last_dispatch;
    int dispatch_count, wb_count, resolve_count, mis_count, idle_count;

    // -----------------------------------------------------------------------
    function new(string name = "rand_constr_seq");
      super.new(name);
    endfunction

    // ========================================================================
    //  body — main loop
    // ========================================================================
    task body();
      int action;

      get_vif();
      begin
        uvm_component comp = uvm_top.find("*.agt.drv");
        if (!$cast(drv, comp)) `uvm_fatal("RAND", "Driver cast failed")
      end

      // Init state
      foreach (tag_alive[i])   tag_alive[i]   = 1'b0;
      foreach (tag_owner[i])   tag_owner[i]   = 7'h0;
      foreach (tag_wb_done[i]) tag_wb_done[i] = 1'b0;
      current_br_mask = 20'h0;
      ftq_counter = 0;
      cycle_since_last_dispatch = 1;
      dispatch_count = 0; wb_count = 0; resolve_count = 0;
      mis_count = 0; idle_count = 0;

      `uvm_info("RAND",
        $sformatf("============== RANDOM ROB SEQ : %0d cycles ==============",
        NUM_CYCLES), UVM_LOW)

      // Wait for reset to clear, ROB sẵn sàng
      repeat (3) @(posedge vif.clock);

      // Main loop
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

      // Drain
      `uvm_info("RAND", "--- Drain phase ---", UVM_LOW)
      drain();

      `uvm_info("RAND",
        $sformatf("Stats: Dispatch=%0d  WB=%0d  Resolve=%0d  Mispredict=%0d  Idle=%0d",
          dispatch_count, wb_count, resolve_count, mis_count, idle_count), UVM_LOW)
      `uvm_info("RAND", "============== RANDOM SEQ COMPLETE ==============", UVM_LOW)
    endtask

    // ========================================================================
    //  pick_action — chọn hành động dựa trạng thái ROB & tracking
    // ========================================================================
    function int pick_action();
      int wd, ww, wr, wm, wi;
      int n_alive_br;
      int n_mis_eligible;
      int total, r;

      n_alive_br     = count_alive_tags();
      n_mis_eligible = count_mispredict_eligible();

      // Default weights
      wd = 40;  // dispatch
      ww = 30;  // writeback
      wr =  8;  // resolve_correct
      wm =  5;  // mispredict
      wi = 17;  // idle

      // [D3] Khi ready=0 → cấm dispatch
      if (!vif.mon_cb.ready) begin
        wd = 0; ww = 50; wm = 0; wr = 0; wi = 50;
      end

      // Không có entry → cấm WB / resolve / mispredict
      if (live_q.size() == 0) begin
        ww = 0; wm = 0; wr = 0;
        if (vif.mon_cb.ready) begin wd = 70; wi = 30; end
        else                  begin wd = 0;  wi = 100; end
      end

      // Empty
      if (vif.mon_cb.empty) begin ww = 0; wm = 0; wr = 0; end

      // Đang rollback / flush → chỉ idle để DUT settle
      if (vif.mon_cb.commit_rollback || vif.mon_cb.flush_valid) begin
        wd = 0; ww = 0; wm = 0; wr = 0; wi = 100;
      end

      // [T1] Vừa dispatch xong → ít nhất 1 cycle mới WB
      if (cycle_since_last_dispatch == 0) ww = 0;

      // Không có branch tag alive → cấm resolve
      if (n_alive_br == 0)     wr = 0;
      // Không có tag chưa WB → cấm mispredict (RULE MỚI)
      if (n_mis_eligible == 0) wm = 0;

      // Cap branch saturation: nếu sắp cạn 20 bit → ép resolve
      if (n_alive_br >= 18) begin
        wd = 0;
        wr = wr + 30;
        wm = wm + ((n_mis_eligible > 0) ? 20 : 0);
      end

      total = wd + ww + wr + wm + wi;
      if (total == 0) return 4;        // idle fallback
      r = $urandom_range(0, total - 1);

      if (r < wd)                  return 0;
      if (r < wd + ww)             return 1;
      if (r < wd + ww + wr)        return 2;
      if (r < wd + ww + wr + wm)   return 3;
                                   return 4;
    endfunction

    // ========================================================================
    //  do_dispatch — dispatch full / partial row tại tail hiện tại của DUT
    //                rob_idx liên tục, ftq_idx CHUNG cho cả row.
    //                Có thể chèn 1 branch instruction tại 1 bank random.
    // ========================================================================
    task do_dispatch();
      rob_transaction tr;
      bit [6:0]  cur_tail;
      int        num_valid;
      bit        have_branch;
      int        br_bank;
      int        new_bit;
      bit [19:0] mask_old;
      bit [19:0] mask_new;

      // Re-check ready (race-safe) — [D3]
      if (!vif.mon_cb.ready) begin
        do_idle();
        return;
      end

      cur_tail  = vif.mon_cb.rob_tail_idx;
      num_valid = $urandom_range(1, 4);
      mask_old  = current_br_mask;
      mask_new  = mask_old;
      new_bit   = -1;
      br_bank   = 0;

      // ~25% có branch trong row, miễn còn bit trống
      have_branch = ($urandom_range(0, 99) < 25);
      if (have_branch) begin
        new_bit = alloc_br_bit();
        if (new_bit < 0) begin
          have_branch = 0;          // 20 bit hết → bỏ branch lần này
        end else begin
          br_bank  = $urandom_range(0, num_valid - 1);
          mask_new = mask_old | (20'h1 << new_bit);
        end
      end

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                = ROB_DISPATCH;
      tr.enq_partial_stall = 1'b0;        // luôn full-cycle dispatch

      for (int b = 0; b < CW; b++) begin
        // Constraint #1 + #2: rob_idx liên tục, ftq_idx chung
        tr.enq_valids[b]              = (b < num_valid);
        tr.enq_uops_rob_idx[b]        = 7'(cur_tail + b);
        tr.enq_uops_ftq_idx[b]        = 6'(ftq_counter);
        tr.enq_uops_pc_lob[b]         = 6'(b * 4);

        // Random uopc, pdst, ldst
        tr.enq_uops_uopc[b]           = 7'($urandom_range(0, 127));
        tr.enq_uops_pdst[b]           = 7'($urandom_range(1, 127));
        tr.enq_uops_stale_pdst[b]     = 7'($urandom_range(0, 127));
        tr.enq_uops_ldst[b]           = 6'($urandom_range(1, 31));
        tr.enq_uops_ldst_val[b]       = 1'b1;
        tr.enq_uops_dst_rtype[b]      = 2'b01;
        tr.enq_uops_fp_val[b]         = 1'b0;
        tr.enq_uops_unsafe[b]         = 1'b1;

        // Constraint #3: per-bank branch mask
        //   bank ≤ br_bank → mask_old (chưa thấy tag mới)
        //   bank >  br_bank → mask_new (đã thấy tag mới)
        if (have_branch) begin
          tr.enq_uops_is_br[b]   = (b == br_bank);
          tr.enq_uops_br_mask[b] = (b <= br_bank) ? mask_old : mask_new;
        end else begin
          tr.enq_uops_is_br[b]   = 1'b0;
          tr.enq_uops_br_mask[b] = mask_old;
        end

        // Random TC này không gen exception/fence/store/unique
        tr.enq_uops_exception[b]       = 1'b0;
        tr.enq_uops_exc_cause[b]       = 64'd0;
        tr.enq_uops_uses_ldq[b]        = 1'b0;
        tr.enq_uops_uses_stq[b]        = 1'b0;
        tr.enq_uops_is_fence[b]        = 1'b0;
        tr.enq_uops_is_fencei[b]       = 1'b0;
        tr.enq_uops_is_unique[b]       = 1'b0;
        tr.enq_uops_flush_on_commit[b] = 1'b0;
      end
      finish_item(tr);
      @(posedge vif.clock);

      // ----------------------------------------------------------------------
      //  Update local tracking
      // ----------------------------------------------------------------------
      for (int b = 0; b < num_valid; b++) begin
        live_entry_t e;
        e.rob_idx     = 7'(cur_tail + b);
        e.pdst        = tr.enq_uops_pdst[b];
        e.br_mask     = tr.enq_uops_br_mask[b];
        e.is_br       = (have_branch && b == br_bank);
        e.br_tag_bit  = (have_branch && b == br_bank) ? new_bit : -1;
        live_q.push_back(e);
      end

      if (have_branch) begin
        tag_alive  [new_bit] = 1'b1;
        tag_owner  [new_bit] = 7'(cur_tail + br_bank);
        tag_wb_done[new_bit] = 1'b0;       //  RULE MỚI: branch chưa WB
        current_br_mask      = mask_new;
      end

      ftq_counter               = (ftq_counter + 1) & 6'h3F;
      cycle_since_last_dispatch = 0;
      dispatch_count++;

      `uvm_info("RAND",
        $sformatf("[D] tail=%0d  nv=%0d  br=%0b  br_bank=%0d  new_bit=%0d  mask=%05h",
          cur_tail, num_valid, have_branch, br_bank, new_bit, current_br_mask),
        UVM_HIGH)
    endtask

    // ========================================================================
    //  do_wb — WB random subset của live_q
    //          Chỉ gửi rob_idx + pdst đã được dispatch (W3 — match shadow).
    //
    //          *** RULE MỚI ***
    //          Nếu entry được WB là 1 lệnh branch (is_br=1, br_tag_bit ≥ 0)
    //          → đánh dấu tag_wb_done[bit]=1 để cấm mispredict bit đó về sau.
    // ========================================================================
    task do_wb();
      rob_transaction tr;
      int n_wb;

      if (live_q.size() == 0) begin do_idle(); return; end

      // 1..min(live_q, 10) entry mỗi cycle (ROB có 10 WB port)
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

        // [W1, W2] kiểm tra qua shadow của driver
        if (drv.can_writeback(le.rob_idx)) begin
          tr.wb_valid[i]   = 1'b1;
          tr.wb_rob_idx[i] = le.rob_idx;
          tr.wb_pdst[i]    = le.pdst;       // [W3] pdst PHẢI khớp dispatch

          // RULE MỚI: nếu entry là branch chưa-resolve → mark tag is WB-done
          if (le.is_br && le.br_tag_bit >= 0
              && tag_alive[le.br_tag_bit]
              && tag_owner[le.br_tag_bit] == le.rob_idx) begin
            tag_wb_done[le.br_tag_bit] = 1'b1;
            `uvm_info("RAND",
              $sformatf("[W] branch rob_idx=%0d (bit %0d) WB-done → mispredict bit cấm",
                le.rob_idx, le.br_tag_bit), UVM_MEDIUM)
          end

          wb_count++;
          `uvm_info("RAND",
            $sformatf("[W] rob_idx=%0d pdst=%0d", le.rob_idx, le.pdst), UVM_HIGH)
        end else begin
          `uvm_info("RAND",
            $sformatf("[W] skip rob_idx=%0d (busy=0 hoặc đã killed)", le.rob_idx),
            UVM_HIGH)
        end

        // Xóa khỏi tracking dù WB thành công hay đã chết
        live_q.delete(pick);
      end

      finish_item(tr);
      @(posedge vif.clock);
      cycle_since_last_dispatch++;
    endtask

    // ========================================================================
    //  do_resolve — resolve_correct với 1 bit ALIVE đã được dispatch.
    //              Cho phép cả tag đã WB-done (typical flow: WB rồi resolve).
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

      // Update tracking: clear bit đó ở mọi entry, free tag
      foreach (live_q[i]) begin
        live_q[i].br_mask &= ~mask;
        if (live_q[i].is_br && live_q[i].br_tag_bit == bit_pos) begin
          live_q[i].is_br      = 1'b0;
          live_q[i].br_tag_bit = -1;
        end
      end
      tag_alive  [bit_pos] = 1'b0;
      tag_wb_done[bit_pos] = 1'b0;
      current_br_mask    &= ~mask;

      cycle_since_last_dispatch++;
      resolve_count++;
      `uvm_info("RAND",
        $sformatf("[R] resolve_correct mask=%05h (bit=%0d)", mask, bit_pos),
        UVM_MEDIUM)
    endtask

    // ========================================================================
    //  do_mispredict — mispredict 1 bit ALIVE & CHƯA WB; kill entries phụ thuộc;
    //                  free chain tag của các branch trẻ hơn cũng bị kill.
    //                  rob_idx truyền vào = tag_owner của bit (lệnh branch).
    //
    //                  *** RULE MỚI ***
    //                  Chỉ chọn từ pool {tag_alive=1 AND tag_wb_done=0}
    // ========================================================================
    task do_mispredict();
      rob_transaction tr;
      int             eligible_bits[$];
      int             pick, bit_pos;
      bit [19:0]      mask;
      bit [6:0]       br_idx;

      get_mispredict_eligible(eligible_bits);
      if (eligible_bits.size() == 0) begin
        `uvm_info("RAND",
          "[M] no eligible bit (all alive tags đã WB) → idle", UVM_HIGH)
        do_idle();
        return;
      end

      pick    = $urandom_range(0, eligible_bits.size() - 1);
      bit_pos = eligible_bits[pick];
      mask    = 20'h1 << bit_pos;
      br_idx  = tag_owner[bit_pos];

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = mask;
      tr.br_mispredict_mask = mask;
      tr.br_rob_idx         = br_idx;       // rob_idx của branch sở hữu tag
      tr.br_mispredict      = 1'b1;
      tr.br_taken           = $urandom_range(0, 1);
      finish_item(tr);
      @(posedge vif.clock);

      // ----------------------------------------------------------------------
      //  Update tracking: kill các entry có br_mask & mask != 0.
      //  Nếu entry bị kill là 1 branch khác → free tag của nó (chain free).
      //  Lệnh branch sở hữu chính bit_pos KHÔNG có bit_pos trong mask của
      //  mình (constraint #3) nên KHÔNG bị kill bởi mask này — nó chỉ bị
      //  resolve.
      // ----------------------------------------------------------------------
      for (int i = live_q.size() - 1; i >= 0; i--) begin
        if (live_q[i].br_mask & mask) begin
          // Nếu là branch khác → free tag của nó
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

      // Free chính bit_pos (branch đã được resolve qua mispredict)
      tag_alive  [bit_pos] = 1'b0;
      tag_wb_done[bit_pos] = 1'b0;
      current_br_mask    &= ~mask;

      // Mark branch instruction đã resolved trong tracking
      foreach (live_q[i]) begin
        if (live_q[i].is_br && live_q[i].br_tag_bit == bit_pos) begin
          live_q[i].is_br      = 1'b0;
          live_q[i].br_tag_bit = -1;
        end
      end

      mis_count++;
      `uvm_info("RAND",
        $sformatf("[M] mispredict bit=%0d mask=%05h br_idx=%0d → live=%0d",
          bit_pos, mask, br_idx, live_q.size()), UVM_MEDIUM)

      // Recovery delay — DUT cần vài cycle để commit_rollback settle
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
    //  update_after_dut_events — phản ứng với các sự kiện ngoài tracking:
    //    - flush_valid (do exception, không gen ở TC này nhưng safety)
    // ========================================================================
    function void update_after_dut_events();
      if (vif.mon_cb.flush_valid) begin
        live_q.delete();
        foreach (tag_alive[i])   tag_alive[i]   = 1'b0;
        foreach (tag_wb_done[i]) tag_wb_done[i] = 1'b0;
        current_br_mask = 20'h0;
        `uvm_info("RAND", "[E] flush_valid → clear all tracking", UVM_MEDIUM)
      end
    endfunction

    // ========================================================================
    //  drain — WB hết entry còn sống, resolve hết tag, chờ ROB rỗng
    // ========================================================================
    task drain();
      int attempts;

      // 1) WB hết live_q
      attempts = 0;
      while (live_q.size() > 0 && attempts < 100) begin
        rob_transaction tr;
        int n_wb;

        @(posedge vif.clock);
        n_wb = (live_q.size() < 10) ? live_q.size() : 10;

        tr = rob_transaction::type_id::create("tr");
        start_item(tr);
        tr.op = ROB_WRITEBACK;
        for (int p = 0; p < NUM_WB_PORTS; p++) tr.wb_valid[p] = 1'b0;

        for (int i = 0; i < n_wb && live_q.size() > 0; i++) begin
          live_entry_t le = live_q[0];
          if (drv.can_writeback(le.rob_idx)) begin
            tr.wb_valid[i]   = 1'b1;
            tr.wb_rob_idx[i] = le.rob_idx;
            tr.wb_pdst[i]    = le.pdst;
            // RULE MỚI: cũng update tag_wb_done khi drain WB branch
            if (le.is_br && le.br_tag_bit >= 0
                && tag_alive[le.br_tag_bit]
                && tag_owner[le.br_tag_bit] == le.rob_idx)
              tag_wb_done[le.br_tag_bit] = 1'b1;
          end
          live_q.delete(0);
        end
        finish_item(tr);
        @(posedge vif.clock);
        attempts++;
      end

      // 2) Resolve_correct cho mọi tag còn alive (free shadow)
      for (int i = 0; i < 20; i++) begin
        if (tag_alive[i]) begin
          rob_transaction tr;
          tr = rob_transaction::type_id::create("tr");
          start_item(tr);
          tr.op                 = ROB_BRANCH_UPDATE;
          tr.br_resolve_mask    = 20'h1 << i;
          tr.br_mispredict_mask = 20'h0;
          tr.br_rob_idx         = 7'h0;
          tr.br_mispredict      = 1'b0;
          finish_item(tr);
          @(posedge vif.clock);
          tag_alive  [i] = 1'b0;
          tag_wb_done[i] = 1'b0;
        end
      end
      current_br_mask = 20'h0;

      // 3) Idle để DUT commit hết
      repeat (40) begin
        do_idle();
        if (vif.mon_cb.empty) begin
          `uvm_info("RAND", "Drain complete: ROB empty", UVM_LOW)
          return;
        end
      end
      if (!vif.mon_cb.empty)
        `uvm_warning("RAND", "ROB not empty after drain — may need more cycles")
    endtask

    // ========================================================================
    //  Helper functions
    // ========================================================================
    function int alloc_br_bit();
      // Cấp phát bit thấp nhất còn trống — constraint #3
      for (int i = 0; i < 20; i++) if (!tag_alive[i]) return i;
      return -1;
    endfunction

    function int count_alive_tags();
      int cnt = 0;
      for (int i = 0; i < 20; i++) if (tag_alive[i]) cnt++;
      return cnt;
    endfunction

    // RULE MỚI: chỉ những bit alive AND chưa WB-done mới mispredict được
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
  //  Test class — đăng ký với UVM factory để chạy bằng +UVM_TESTNAME
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
