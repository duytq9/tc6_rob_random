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
//   6. *** FENCE & EXCEPTION (rev 3) ***
//      Mỗi row dispatch là 1 trong 5 loại loại trừ lẫn nhau:
//          ROW_NORMAL        60% — flow bình thường
//          ROW_BRANCH        25% — như rev2
//          ROW_FENCE          7% — is_fence=1, unsafe=0, single-bank
//          ROW_FENCE_UNIQUE   4% — is_fence=1, is_unique=1, unsafe=0
//          ROW_EXCEPTION      4% — 1 bank gắn exception=1, exc_cause ≠ 0
//
//      Cách handle:
//      - FENCE / FENCE_UNIQUE:
//          DUT shadow set busy=0 ngay khi dispatch (e[idx].busy = is_fence?0:1).
//          Nếu gửi WB cho fence → trigger W2 double-WB → KHÔNG add live_q.
//          Fence tự commit khi reach head. Với is_unique=1 (fence.i,
//          sfence.vma) DUT vào FSM s_wait_till_empty → vif.ready=0 cho đến
//          khi ROB drain. Vòng pick_action gặp !ready → idle, tự thoát khi
//          ready trở lại 1.
//      - EXCEPTION:
//          Entry busy=1 bình thường, CẦN WB như entry thường. Sau commit,
//          DUT raise com_xcpt_valid + flush_valid → kill toàn bộ younger
//          entry, head/tail reset. update_after_dut_events đã sẵn handle:
//          flush_valid → clear live_q + tag_alive + tag_wb_done.
//
//   7. *** BANK-ALIGNMENT (rev 4) ***
//      Sau rollback từ mispredict / flush, rob_tail_idx có thể rơi giữa
//      row 4-bank (cur_tail % 4 != 0). Cycle hiện tại chỉ được dispatch
//      tối đa (4 - cur_tail % 4) lệnh để FILL NỐT row đó; cycle sau
//      tail align mod-4 và dispatch toàn-row trở lại.
//          bank_offset   = cur_tail % 4
//          max_remaining = 4 - bank_offset
//          num_valid    ← min(num_valid_random, max_remaining)
//      ví dụ: tail=46 → off=2 → cap num_valid ≤ 2 (rob_idx 46,47)
//             cycle sau start từ tail=48 → off=0 → tự do 1..4.
//
//   8. *** FLUSH COOLDOWN (rev 5, bumped rev 7) ***
//      Khi DUT raise flush_valid (do exception commit, hoặc DUT-level
//      flush), rob_tail_idx cần ~3 chu kỳ để được cập nhật về vị trí
//      mới. Nếu trong khoảng đó cố dispatch → tail SAI → entry rác.
//      → Tracking thêm flush_wait_cycles, re-arm = 6 mỗi lần
//        flush_valid được detect (3 cycle margin trên 3 cycle HW để
//        chịu được nhiễu từ partial-stall / back-to-back event).
//      → update_after_dut_events giảm counter mỗi iter.
//      → pick_action thấy counter > 0 → ép wi=100, cấm mọi action khác.
//
//   9. *** SAME-PDST OLDER-FIRST (rev 6) ***
//      Nếu có ≥2 live entry chia sẻ cùng pdst, LUÔN WB entry có
//      rob_idx CŨ hơn (theo program order) trước.
//      → live_q được push_back theo dispatch order, vị trí trong queue
//        chính là program order (KHÔNG dùng so sánh rob_idx số học,
//        vì rob_idx 7-bit có wrap-around).
//      → do_wb pick random như cũ, sau đó nếu thấy entry khác cùng
//        pdst ở vị trí queue sớm hơn → SWITCH pick về entry sớm nhất.
//      → Logic tự respect ngay cả khi WB nhiều port trong cùng cycle.
//
//  10. *** EXCEPTION WB PENDING (rev 8) ***
//      Vấn đề rev 5/7: flush_wait_cycles được re-arm ở
//      update_after_dut_events khi flush_valid được sample. Nhưng
//      do_mispredict tiêu 4-9 cycle (repeat 3-8 do_idle), giữa lúc
//      đó flush_valid pulse 1-cycle có thể raise và tắt → MISS.
//      Solution: trigger PROACTIVE từ điểm WB:
//      → live_entry_t.has_exception đã track từ dispatch (rev 3).
//      → do_wb: khi WB thành công entry với has_exception=1
//        → set exception_wb_pending = 1 NGAY tại điểm này
//          (deterministic, không phụ thuộc timing flush_valid pulse).
//      → pick_action khi pending=1: cấm wd & wm
//        (do_dispatch tail rác, do_mispredict multi-cycle).
//        Cho phép do_wb / do_resolve / do_idle (đều 1-cycle)
//        → ROB drain commit → exception entry reach head → DUT raise
//          flush_valid → update_after_dut_events bắt được CHÍNH XÁC
//          (vì mỗi iter trong phase pending = 1 cycle, không miss).
//      → flush_valid clear pending → chuyển sang phase flush_wait_cycles
//        chờ tail update (rule #8).
//      → Safety: timeout 200 iter tự clear pending (đề phòng entry bị
//        flush_valid khác kill — VD mispredict ngược kill exception
//        entry younger hơn branch).
//
//  11. *** UNIQUE PDST POOL (rev 9) ***
//      Mỗi entry đã dispatch nhưng chưa WB chiếm 1 slot pdst trong
//      {1..127}. Row mới bắt buộc:
//        (a) các bank trong cùng row có pdst KHÁC NHAU
//        (b) không trùng pdst của bất kỳ entry còn live trong live_q
//      → do_dispatch: build avail_pool = {1..127} \ live_q.pdst.
//        - Pool đầy (live_q chiếm 127) → early-return do_idle, đợi WB.
//        - Pool < num_valid → cap num_valid theo pool.
//        - Pick num_valid pdst unique từ pool (random shuffle, no replace).
//      → Fence / fence_unique KHÔNG vào live_q (busy=0 từ dispatch),
//        bypass logic pool, pdst random như cũ.
//      → pick_action thêm hint: live_q.size() >= 127 → cấm wd,
//        boost ww để drain pool nhanh.
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
      bit        has_exception; // entry mang flag exception
    } live_entry_t;

    live_entry_t live_q[$];

    // -----------------------------------------------------------------------
    //  Row dispatch type — mutual exclusion theo ảnh hướng dẫn
    //     NORMAL=60%  BRANCH=25%  FENCE=7%  FENCE_UNIQUE=4%  EXCEPTION=4%
    // -----------------------------------------------------------------------
    typedef enum int {
      ROW_NORMAL        = 0,
      ROW_BRANCH        = 1,
      ROW_FENCE         = 2,
      ROW_FENCE_UNIQUE  = 3,
      ROW_EXCEPTION     = 4
    } row_type_e;

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
    int fence_count, fence_unique_count, exception_count;
    int flush_wait_cycles;     // cooldown sau flush_valid để chờ rob_tail_idx update
    bit exception_wb_pending;  // = 1 từ lúc WB exception entry → đến khi flush_valid raise
    int exception_wait_count;  // safety timeout counter cho exception_wb_pending

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
      fence_count = 0; fence_unique_count = 0; exception_count = 0;
      flush_wait_cycles = 0;
      exception_wb_pending = 1'b0;
      exception_wait_count = 0;

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
      `uvm_info("RAND",
        $sformatf("       Fence=%0d  FenceUnique=%0d  Exception=%0d",
          fence_count, fence_unique_count, exception_count), UVM_LOW)
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

      // Đang rollback / flush / cooldown → chỉ idle để DUT settle.
      // flush_wait_cycles > 0 nghĩa là vừa flush_valid, rob_tail_idx chưa
      // kịp update → nếu dispatch sẽ dùng tail sai.
      if (vif.mon_cb.commit_rollback || vif.mon_cb.flush_valid
          || flush_wait_cycles > 0) begin
        wd = 0; ww = 0; wm = 0; wr = 0; wi = 100;
      end

      // *** EXCEPTION WB PENDING (rev 8) ***
      // Đã WB exception entry, đang chờ entry reach head + DUT raise
      // flush_valid. Cấm các action có thể "ăn" nhiều cycle hoặc làm
      // rác state:
      //   - do_dispatch (wd): tail sắp reset, dispatch giờ tạo entry rác.
      //   - do_mispredict (wm): repeat 3-8 cycle idle bên trong → có thể
      //     miss flush_valid pulse 1-cycle.
      // Cho phép wb / resolve / idle (đều 1-cycle) để ROB drain commit
      // các entries older, đẩy exception entry tới head.
      // Mỗi iter trong phase này = 1 cycle → update_after_dut_events
      // sẽ KHÔNG miss flush_valid khi nó raise.
      else if (exception_wb_pending) begin
        wd = 0; wm = 0;
      end

      // [T1] Vừa dispatch xong → ít nhất 1 cycle mới WB
      if (cycle_since_last_dispatch == 0) ww = 0;

      // *** PDST POOL FULL HINT (rev 9) ***
      // live_q chiếm hết 127 pdst → dispatch sẽ chỉ early-return idle.
      // Cấm wd, ép ww/wi để pool drain qua WB.
      if (live_q.size() >= 127) begin
        wd = 0;
        if (ww == 0) ww = 30;   // unstuck: cho phép WB dù vừa dispatch xong
      end

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
    //  pick_row_type — chọn 1 trong 5 loại dispatch row theo phân phối:
    //     NORMAL=60%  BRANCH=25%  FENCE=7%  FENCE_UNIQUE=4%  EXCEPTION=4%
    // ========================================================================
    function row_type_e pick_row_type();
      int r = $urandom_range(0, 99);
      if (r < 60) return ROW_NORMAL;
      if (r < 85) return ROW_BRANCH;        // 60..84  (25%)
      if (r < 92) return ROW_FENCE;         // 85..91  (7%)
      if (r < 96) return ROW_FENCE_UNIQUE;  // 92..95  (4%)
      return ROW_EXCEPTION;                 // 96..99  (4%)
    endfunction

    // ========================================================================
    //  do_dispatch — dispatch full / partial row tại tail hiện tại của DUT
    //                rob_idx liên tục, ftq_idx CHUNG cho cả row.
    //                Row là 1 trong 5 loại: NORMAL / BRANCH / FENCE /
    //                FENCE_UNIQUE / EXCEPTION (mutual exclusive).
    // ========================================================================
    task do_dispatch();
      rob_transaction tr;
      bit [6:0]  cur_tail;
      int        num_valid;
      row_type_e row_type;

      // Branch fields
      int        br_bank;
      int        new_bit;
      bit [19:0] mask_old;
      bit [19:0] mask_new;

      // Exception fields
      int        exc_bank;

      // Pdst pool (rev 9): unique pdst pick từ {1..127} \ live_q.pdst
      int        pdst_picked[];

      // Re-check ready (race-safe) — [D3]
      if (!vif.mon_cb.ready) begin
        do_idle();
        return;
      end

      cur_tail  = vif.mon_cb.rob_tail_idx;
      mask_old  = current_br_mask;
      mask_new  = mask_old;
      new_bit   = -1;
      br_bank   = 0;
      exc_bank  = 0;

      row_type  = pick_row_type();

      // -------------------------------------------------------------
      //  Branch tag pool guard: nếu định ROW_BRANCH nhưng pool đầy
      //  → downgrade thành ROW_NORMAL.
      // -------------------------------------------------------------
      if (row_type == ROW_BRANCH) begin
        new_bit = alloc_br_bit();
        if (new_bit < 0) begin
          row_type = ROW_NORMAL;
        end else begin
          mask_new = mask_old | (20'h1 << new_bit);
        end
      end

      // -------------------------------------------------------------
      //  num_valid theo từng loại:
      //   FENCE / FENCE_UNIQUE → single-bank (bank 0) cho clean
      //   Còn lại              → 1..4 bank random
      // -------------------------------------------------------------
      case (row_type)
        ROW_FENCE, ROW_FENCE_UNIQUE: num_valid = 1;
        default:                     num_valid = $urandom_range(1, 4);
      endcase

      // -------------------------------------------------------------
      //  *** BANK-ALIGNMENT GUARD (rev 4) ***
      //  Sau rollback từ mispredict / flush, rob_tail_idx có thể rơi
      //  giữa row (không chia hết cho 4). Cycle này chỉ dispatch để
      //  FILL NỐT row hiện tại; cycle sau tail sẽ align mod-4 như
      //  thường.
      //     bank_offset   = cur_tail % 4
      //     max_remaining = 4 - bank_offset
      //     num_valid    ← min(num_valid, max_remaining)
      //  ví dụ: tail=46 → offset=2 → cap num_valid ≤ 2
      //          (dispatch rob_idx 46,47 → cycle sau start từ 48)
      // -------------------------------------------------------------
      begin
        int bank_offset;
        int max_remaining;
        bank_offset   = int'(cur_tail & 2'h3);
        max_remaining = 4 - bank_offset;
        if (num_valid > max_remaining) begin
          `uvm_info("RAND",
            $sformatf("[D] tail=%0d mis-aligned (off=%0d) → cap num_valid %0d→%0d",
              cur_tail, bank_offset, num_valid, max_remaining), UVM_HIGH)
          num_valid = max_remaining;
        end
      end

      // -------------------------------------------------------------
      //  *** UNIQUE PDST POOL (rev 9) ***
      //  Mỗi entry trong live_q (đã dispatch, chưa WB) chiếm 1 slot
      //  pdst trong {1..127}. Row mới phải pick pdst:
      //   (a) khác nhau giữa các bank trong cùng row
      //   (b) không trùng pdst của bất kỳ entry còn live trong live_q
      //  Nếu pool đầy (live_q chiếm hết 127 pdst) → đợi WB drain.
      //  Nếu pool còn < num_valid → cap num_valid theo pool.
      //
      //  Fence / fence_unique KHÔNG vào live_q (busy=0 từ dispatch),
      //  pdst của fence không "chiếm slot" → giữ random như cũ,
      //  bypass logic pool.
      // -------------------------------------------------------------
      if (row_type != ROW_FENCE && row_type != ROW_FENCE_UNIQUE) begin
        bit [127:0] used_pdst_mask;
        int         avail_pool[$];

        used_pdst_mask = '0;
        foreach (live_q[k]) used_pdst_mask[live_q[k].pdst] = 1'b1;
        for (int p = 1; p <= 127; p++)
          if (!used_pdst_mask[p]) avail_pool.push_back(p);

        // Pool đầy hoàn toàn → drain WB trước, idle skip dispatch
        if (avail_pool.size() == 0) begin
          `uvm_info("RAND",
            $sformatf("[D] pdst pool đầy (live=%0d/127) → drain WB, skip dispatch",
              live_q.size()), UVM_MEDIUM)
          do_idle();
          return;
        end

        // Cap num_valid theo pool nếu cần
        if (num_valid > avail_pool.size()) begin
          `uvm_info("RAND",
            $sformatf("[D] cap num_valid theo pool: %0d → %0d (live=%0d)",
              num_valid, avail_pool.size(), live_q.size()), UVM_HIGH)
          num_valid = avail_pool.size();
        end

        // Pick num_valid pdst unique từ pool (random shuffle, không lặp)
        pdst_picked = new[num_valid];
        for (int b = 0; b < num_valid; b++) begin
          int idx;
          idx              = $urandom_range(0, avail_pool.size() - 1);
          pdst_picked[b]   = avail_pool[idx];
          avail_pool.delete(idx);
        end
      end

      if (row_type == ROW_BRANCH)
        br_bank  = $urandom_range(0, num_valid - 1);
      if (row_type == ROW_EXCEPTION)
        exc_bank = $urandom_range(0, num_valid - 1);

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                = ROB_DISPATCH;
      tr.enq_partial_stall = 1'b0;        // luôn full-cycle dispatch

      for (int b = 0; b < CW; b++) begin
        // Default flags — set hết, override theo row_type bên dưới
        tr.enq_valids[b]              = (b < num_valid);
        tr.enq_uops_rob_idx[b]        = 7'(cur_tail + b);
        tr.enq_uops_ftq_idx[b]        = 6'(ftq_counter);
        tr.enq_uops_pc_lob[b]         = 6'(b * 4);

        tr.enq_uops_uopc[b]           = 7'($urandom_range(0, 127));
        // Pdst: unique pool pick cho NORMAL/BRANCH/EXCEPTION valid bank.
        //       Random ok cho fence (không vào live_q) và bank invalid.
        if (b < num_valid
            && row_type != ROW_FENCE
            && row_type != ROW_FENCE_UNIQUE) begin
          tr.enq_uops_pdst[b]         = 7'(pdst_picked[b]);
        end else begin
          tr.enq_uops_pdst[b]         = 7'($urandom_range(1, 127));
        end
        tr.enq_uops_stale_pdst[b]     = 7'($urandom_range(0, 127));
        tr.enq_uops_ldst[b]           = 6'($urandom_range(1, 31));
        tr.enq_uops_ldst_val[b]       = 1'b1;
        tr.enq_uops_dst_rtype[b]      = 2'b01;
        tr.enq_uops_fp_val[b]         = 1'b0;
        tr.enq_uops_unsafe[b]         = 1'b1;

        tr.enq_uops_is_br[b]          = 1'b0;
        tr.enq_uops_br_mask[b]        = mask_old;

        tr.enq_uops_exception[b]       = 1'b0;
        tr.enq_uops_exc_cause[b]       = 64'd0;
        tr.enq_uops_uses_ldq[b]        = 1'b0;
        tr.enq_uops_uses_stq[b]        = 1'b0;
        tr.enq_uops_is_fence[b]        = 1'b0;
        tr.enq_uops_is_fencei[b]       = 1'b0;
        tr.enq_uops_is_unique[b]       = 1'b0;
        tr.enq_uops_flush_on_commit[b] = 1'b0;

        // Per row-type override (chỉ trên các bank valid)
        if (b < num_valid) begin
          case (row_type)
            ROW_BRANCH: begin
              // Constraint #3: per-bank branch mask
              tr.enq_uops_is_br[b]   = (b == br_bank);
              tr.enq_uops_br_mask[b] = (b <= br_bank) ? mask_old : mask_new;
            end

            ROW_FENCE: begin
              // Plain fence (memfence): is_fence=1, unsafe=0 (theo helper)
              // DUT shadow set busy=0 ngay từ dispatch → KHÔNG WB sau này
              tr.enq_uops_is_fence[b] = 1'b1;
              tr.enq_uops_unsafe[b]   = 1'b0;
            end

            ROW_FENCE_UNIQUE: begin
              // fence.i / sfence.vma: is_fence=1 + is_unique=1, unsafe=0
              // DUT vào FSM s_wait_till_empty → ready=0 cho tới khi drain
              tr.enq_uops_is_fence[b]  = 1'b1;
              tr.enq_uops_is_unique[b] = 1'b1;
              tr.enq_uops_unsafe[b]    = 1'b0;
            end

            ROW_EXCEPTION: begin
              // Đúng 1 bank gắn exception. Entry busy=1 bình thường, cần WB.
              // Khi commit, DUT raise com_xcpt_valid + flush_valid.
              if (b == exc_bank) begin
                tr.enq_uops_exception[b] = 1'b1;
                tr.enq_uops_exc_cause[b] = 64'($urandom_range(1, 15));
              end
            end

            default: ; // ROW_NORMAL — giữ nguyên defaults
          endcase
        end
      end
      finish_item(tr);
      @(posedge vif.clock);

      // ----------------------------------------------------------------------
      //  Update local tracking
      //   - FENCE / FENCE_UNIQUE: KHÔNG add live_q (busy=0 từ dispatch)
      //   - Còn lại: add live_q như cũ + has_exception flag nếu cần
      // ----------------------------------------------------------------------
      if (row_type != ROW_FENCE && row_type != ROW_FENCE_UNIQUE) begin
        for (int b = 0; b < num_valid; b++) begin
          live_entry_t e;
          e.rob_idx       = 7'(cur_tail + b);
          e.pdst          = tr.enq_uops_pdst[b];
          e.br_mask       = tr.enq_uops_br_mask[b];
          e.is_br         = (row_type == ROW_BRANCH    && b == br_bank);
          e.br_tag_bit    = (row_type == ROW_BRANCH    && b == br_bank)
                              ? new_bit : -1;
          e.has_exception = (row_type == ROW_EXCEPTION && b == exc_bank);
          live_q.push_back(e);
        end
      end

      if (row_type == ROW_BRANCH) begin
        tag_alive  [new_bit] = 1'b1;
        tag_owner  [new_bit] = 7'(cur_tail + br_bank);
        tag_wb_done[new_bit] = 1'b0;       //  RULE MỚI: branch chưa WB
        current_br_mask      = mask_new;
      end

      ftq_counter               = (ftq_counter + 1) & 6'h3F;
      cycle_since_last_dispatch = 0;
      dispatch_count++;

      // Specific row-type counters
      case (row_type)
        ROW_FENCE:        fence_count++;
        ROW_FENCE_UNIQUE: fence_unique_count++;
        ROW_EXCEPTION:    exception_count++;
        default: ;
      endcase

      `uvm_info("RAND",
        $sformatf("[D] type=%-16s tail=%0d nv=%0d br_bank=%0d exc_bank=%0d new_bit=%0d mask=%05h",
          row_type.name(), cur_tail, num_valid, br_bank, exc_bank, new_bit,
          current_br_mask),
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
        int          orig_pick;
        live_entry_t le;

        if (live_q.size() == 0) break;
        pick      = $urandom_range(0, live_q.size() - 1);
        orig_pick = pick;

        // ------------------------------------------------------------------
        //  *** SAME-PDST OLDER-FIRST (rev 6) ***
        //  Nếu có entry khác CÙNG pdst nhưng dispatch SỚM HƠN
        //  (= index nhỏ hơn trong live_q, vì push_back theo program order),
        //  switch pick về entry sớm nhất đó.
        //  Không dùng so sánh rob_idx số học vì rob_idx 7-bit có wrap.
        // ------------------------------------------------------------------
        for (int j = 0; j < pick; j++) begin
          if (live_q[j].pdst == live_q[pick].pdst) begin
            `uvm_info("RAND",
              $sformatf("[W] same-pdst=%0d: switch pick %0d→%0d (older rob_idx=%0d trước rob_idx=%0d)",
                live_q[pick].pdst, orig_pick, j,
                live_q[j].rob_idx, live_q[pick].rob_idx), UVM_HIGH)
            pick = j;
            break;     // j đầu tiên match chính là oldest có cùng pdst
          end
        end

        le = live_q[pick];

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

          // *** EXCEPTION WB DETECTION (rev 8) ***
          // Khi WB entry exception thành công → entry busy=0, exception sẽ
          // reach head và trigger flush_valid. Set pending flag ngay tại đây
          // để pick_action chuyển sang chế độ "an toàn" (chỉ 1-cycle action),
          // tránh bị do_mispredict ăn nhiều cycle khiến miss flush_valid pulse.
          if (le.has_exception) begin
            exception_wb_pending = 1'b1;
            exception_wait_count = 0;
            `uvm_info("RAND",
              $sformatf("[W] EXCEPTION WB rob_idx=%0d → pending=1, chờ flush_valid",
                le.rob_idx), UVM_MEDIUM)
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
    //    - flush_valid: do exception commit hoặc DUT-level flush.
    //      Khi exception entry reach head + busy=0, DUT raise
    //      com_xcpt_valid + flush_valid → kill toàn bộ younger entry,
    //      head/tail reset. Ta clear sạch tracking để đồng bộ shadow.
    //
    //  *** FLUSH COOLDOWN (rev 5, bumped rev 7) ***
    //  Sau khi flush_valid được detect, rob_tail_idx của DUT cần ~3 chu kỳ
    //  để được cập nhật về vị trí mới. Trong khoảng đó nếu cố dispatch sẽ
    //  dùng tail SAI.
    //  → Re-arm flush_wait_cycles = 6 (3 cycle margin trên 3 cycle hardware
    //    để chịu được nhiễu từ partial-stall / back-to-back event).
    //  → Mỗi lần được gọi (1 lần/iter), counter giảm 1.
    //  → pick_action thấy counter > 0 sẽ ép idle, không dispatch.
    //
    //  *** EXCEPTION PENDING TIMEOUT (rev 8) ***
    //  exception_wb_pending được set tại do_wb khi WB entry exception.
    //  Bình thường flush_valid sẽ raise sau vài cycle (entry reach head
    //  → DUT commit → flush). Safety: nếu sau 200 iter vẫn chưa thấy
    //  flush_valid (entry có thể đã bị flushed bởi mispredict trước đó?),
    //  log warning và clear flag để tránh stuck vĩnh viễn.
    // ========================================================================
    function void update_after_dut_events();
      // Tick down flush cooldown mỗi iter (trước khi check flush mới)
      if (flush_wait_cycles > 0) flush_wait_cycles--;

      // Tick exception-wait counter; safety auto-clear nếu quá lâu
      if (exception_wb_pending) begin
        exception_wait_count++;
        if (exception_wait_count > 200) begin
          `uvm_warning("RAND",
            $sformatf("[E] exception_wb_pending timeout sau %0d iter mà không thấy flush_valid → safety-clear",
              exception_wait_count))
          exception_wb_pending = 1'b0;
          exception_wait_count = 0;
        end
      end

      if (vif.mon_cb.flush_valid) begin
        live_q.delete();
        foreach (tag_alive[i])   tag_alive[i]   = 1'b0;
        foreach (tag_wb_done[i]) tag_wb_done[i] = 1'b0;
        current_br_mask      = 20'h0;
        flush_wait_cycles    = 6;     // re-arm: ép idle 6 iter để rob_tail_idx update
        exception_wb_pending = 1'b0;  // flush đã raise → release pending
        exception_wait_count = 0;
        `uvm_info("RAND",
          "[E] flush_valid → clear tracking + cooldown 6 iter (chờ rob_tail_idx update)",
          UVM_MEDIUM)
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
