// ============================================================================
// TC6: Constrained-Random Stimulus (REVISED v3)
//
// v3 fix các deadlock không writeback được:
//   * BUG1: pick_action() trước đây có `if(cycle_since_last_dispatch==0) ww=0`
//     => ngay sau dispatch, ww luôn=0, vì pick kế tiếp đa số lại dispatch
//     (reset cycle về 0 tiếp), WB không bao giờ được pick -> DEADLOCK.
//     Fix: bỏ guard cycle hoàn toàn. Biến cycle_since_last_dispatch đã xóa.
//
//   * BUG2: do_random_writeback/drain xóa inflight kể cả khi can_writeback=0
//     => mất dấu uop, không thể WB lại sau này.
//     Fix: thu thập `eligible[]` = những uop thật sự can_writeback, chỉ
//     remove khi tx.wb_valid[i]=1.
//
//   * BUG3: pick_action dùng `vif.mon_cb.empty` (sampled trước posedge,
//     không phản ánh dispatch mới). Thay bằng tracking cục bộ inflight_order.
//
//   + Thêm "drain pressure": khi inflight_order > 3/4 ROB, giảm dispatch
//     tăng WB để tránh ROB đầy rồi kẹt.
// ============================================================================
class tc6_random_seq extends uvm_sequence #(rob_transaction);
  `uvm_object_utils(tc6_random_seq)

  virtual rob_if.MON vif;
  rob_driver         drv;

  // ---- Config ------------------------------------------------------------
  int unsigned NUM_ACTIONS     = 500;
  int unsigned MAX_ROB_IDX     = NUM_ROB_ENTRIES;  // = CW * NUM_ROB_ROWS
  int unsigned BR_PROB_PERMIL  = 100;              // 10.0% row có branch mới
  int unsigned MIS_PROB_PERMIL = 300;              // 30.0% resolve là mispredict

  // ---- Pointer tracking --------------------------------------------------
  bit [6:0]    pred_tail_idx   = 0;
  bit [6:0]    pred_head_idx   = 0;
  int unsigned dispatch_rounds = 0;
  int unsigned commit_rounds   = 0;

  // ---- Branch-bit allocator (20 bit br_mask) -----------------------------
  bit        br_bit_valid   [20];
  bit [6:0]  br_bit_rob_idx [20];
  bit [19:0] current_br_mask = 20'h0;

  // ---- In-flight uop info (parallel assoc arrays, key = rob_idx) ---------
  bit [6:0]  if_pdst   [bit[6:0]];
  bit [6:0]  if_uopc   [bit[6:0]];
  bit [19:0] if_brmask [bit[6:0]];
  bit        if_is_br  [bit[6:0]];
  bit [4:0]  if_brbit  [bit[6:0]];
  bit [6:0]  inflight_order [$];

  int ftq = 0;

  // ---- Stats (để debug) --------------------------------------------------
  int n_dispatch = 0, n_wb = 0, n_br_correct = 0, n_br_mispred = 0, n_idle = 0;

  function new(string name="tc6_random_seq"); super.new(name); endfunction

  // =======================================================================
  // Helpers
  // =======================================================================
  function int find_free_br_bit();
    for (int i=0; i<20; i++) if (!br_bit_valid[i]) return i;
    return -1;
  endfunction

  function automatic bit [6:0] wrap_inc_row_tail(bit [6:0] idx);
    bit [6:0] nxt = idx + CW;
    if (nxt >= MAX_ROB_IDX) begin
      nxt = nxt - MAX_ROB_IDX;
      dispatch_rounds++;
    end
    return nxt;
  endfunction

  function void remove_inflight(bit [6:0] rid);
    if (!if_pdst.exists(rid)) return;
    if_pdst.delete(rid);
    if_uopc.delete(rid);
    if_brmask.delete(rid);
    if_is_br.delete(rid);
    if_brbit.delete(rid);
    foreach (inflight_order[j]) begin
      if (inflight_order[j] == rid) begin
        inflight_order.delete(j);
        break;
      end
    end
  endfunction

  task clear_brmask_bit(int bitpos);
    bit [6:0]  keys [$];
    bit [19:0] clr = ~(20'b1 << bitpos);
    foreach (if_brmask[k]) keys.push_back(k);
    foreach (keys[i]) if_brmask[keys[i]] &= clr;
  endtask

  task kill_inflight_with_bit(int bitpos);
    bit [6:0]  victims [$];
    bit [19:0] m = 20'b1 << bitpos;
    foreach (if_brmask[k]) begin
      if (if_brmask[k] & m) victims.push_back(k);
    end
    foreach (victims[i]) remove_inflight(victims[i]);
  endtask

  task flush_tracking();
    if_pdst.delete();
    if_uopc.delete();
    if_brmask.delete();
    if_is_br.delete();
    if_brbit.delete();
    inflight_order.delete();
    for (int i=0; i<20; i++) br_bit_valid[i] = 1'b0;
    current_br_mask = 20'h0;
    pred_tail_idx   = 7'h0;
    pred_head_idx   = 7'h0;
  endtask

  // =======================================================================
  // body
  // =======================================================================
  task body();
    int action, act_done;

    if(!uvm_config_db#(virtual rob_if.MON)::get(null,"","vif",vif))
      `uvm_fatal("TC6","No vif")
    begin
      uvm_component comp = uvm_top.find("*.agt.drv");
      if(!$cast(drv, comp)) `uvm_fatal("TC6","Driver cast failed")
    end

    @(posedge vif.clock);
    pred_tail_idx = vif.mon_cb.rob_tail_idx;
    pred_head_idx = vif.mon_cb.rob_head_idx;

    `uvm_info("TC6",
      $sformatf("===== TC6: RANDOM (%0d ACTIONS) =====", NUM_ACTIONS), UVM_LOW)

    act_done = 0;
    while (act_done < NUM_ACTIONS) begin
      action = pick_action();
      case(action)
        0: do_random_dispatch();
        1: do_random_writeback();
        2: do_random_br_update();
        3: do_idle();
      endcase
      act_done++;
      sync_pointers_from_dut();

      // In tiến độ mỗi 100 action
      if ((act_done % 100) == 0) begin
        `uvm_info("TC6",
          $sformatf("Progress %0d/%0d | DIS=%0d WB=%0d BR_OK=%0d BR_MIS=%0d IDLE=%0d | inflight=%0d br_mask=%0h",
                    act_done, NUM_ACTIONS, n_dispatch, n_wb, n_br_correct,
                    n_br_mispred, n_idle, inflight_order.size(), current_br_mask),
          UVM_LOW)
      end
    end

    `uvm_info("TC6",
      $sformatf("--- Drain (DIS=%0d WB=%0d BR_OK=%0d BR_MIS=%0d IDLE=%0d inflight=%0d) ---",
                n_dispatch, n_wb, n_br_correct, n_br_mispred, n_idle,
                inflight_order.size()), UVM_LOW)
    drain_rob();
    `uvm_info("TC6","========== TC6 COMPLETE ==========",UVM_LOW)
  endtask

  // =======================================================================
  task sync_pointers_from_dut();
    bit [6:0] dut_tail = vif.mon_cb.rob_tail_idx;
    bit [6:0] dut_head = vif.mon_cb.rob_head_idx;
    if (dut_head < pred_head_idx && !vif.mon_cb.flush_valid) commit_rounds++;
    if (dut_tail !== pred_tail_idx) pred_tail_idx = dut_tail;
    if (dut_head !== pred_head_idx) pred_head_idx = dut_head;
  endtask

  // =======================================================================
  // pick_action — DÙNG TRACKING CỤC BỘ, không dùng mon_cb.empty,
  //               KHÔNG có guard cycle_since_last_dispatch.
  // =======================================================================
  function int pick_action();
    int wd = 35, ww = 35, wm = 10, wi = 20;
    int tot, r;
    int qsize = inflight_order.size();
    int hi_wm = (MAX_ROB_IDX * 3) / 4;  // 75% threshold

    // Hard override: đang rollback/flush => chỉ idle
    if (vif.mon_cb.commit_rollback || vif.mon_cb.flush_valid) return 3;

    // Soft conditions
    if (!vif.mon_cb.ready)          wd = 0;  // ROB full: không dispatch
    if (qsize == 0)                 ww = 0;  // không có gì để WB
    if (current_br_mask == 20'h0)   wm = 0;  // không có branch in-flight

    // Drain pressure: nếu ROB đang full => boost WB, giảm dispatch
    if (qsize >= hi_wm) begin
      wd = wd / 3;
      ww = (ww == 0) ? 0 : ww * 2;
    end else if (qsize == 0) begin
      // Khởi động: ưu tiên dispatch để có cái mà WB
      wd = 70;
      wi = 30;
    end

    tot = wd + ww + wm + wi;
    if (tot == 0) return 3;
    r = $urandom_range(0, tot-1);
    if (r < wd)            return 0;
    if (r < wd+ww)         return 1;
    if (r < wd+ww+wm)      return 2;
    return 3;
  endfunction

  // =======================================================================
  // DISPATCH
  // =======================================================================
  task do_random_dispatch();
    rob_transaction tr;
    bit [6:0]  cur_tail;
    int        nv, br_bank, new_br_bit;
    bit        include_new_br;
    bit [19:0] new_bit_mask;

    if(!vif.mon_cb.ready) begin do_idle(); return; end

    cur_tail = vif.mon_cb.rob_tail_idx;
    nv       = $urandom_range(1,4);

    include_new_br = 1'b0;
    br_bank        = -1;
    new_br_bit     = -1;
    new_bit_mask   = 20'h0;
    if ($urandom_range(1,1000) <= BR_PROB_PERMIL) begin
      new_br_bit = find_free_br_bit();
      if (new_br_bit >= 0) begin
        br_bank        = $urandom_range(0, nv-1);
        new_bit_mask   = 20'b1 << new_br_bit;
        include_new_br = 1'b1;
      end
    end

    tr = rob_transaction::type_id::create("tr");
    start_item(tr);
    tr.op                = ROB_DISPATCH;
    tr.enq_partial_stall = 0;

    for (int b=0; b<4; b++) begin
      bit [6:0]  this_idx    = (cur_tail + b) % MAX_ROB_IDX;
      bit        this_is_br  = 1'b0;
      bit [19:0] this_brmask = current_br_mask;

      if (b < nv) begin
        if (include_new_br && b == br_bank) begin
          this_is_br  = 1'b1;
          this_brmask = current_br_mask;
        end else if (include_new_br && b > br_bank) begin
          this_is_br  = 1'b0;
          this_brmask = current_br_mask | new_bit_mask;
        end else begin
          this_is_br  = 1'b0;
          this_brmask = current_br_mask;
        end
      end

      tr.enq_valids[b]               = (b < nv);
      tr.enq_uops_rob_idx[b]         = this_idx;
      tr.enq_uops_ftq_idx[b]         = 6'(ftq);
      tr.enq_uops_pc_lob[b]          = 6'(b*4);
      tr.enq_uops_uopc[b]            = 7'($urandom_range(0,127));
      tr.enq_uops_pdst[b]            = 7'($urandom_range(1,127));
      tr.enq_uops_stale_pdst[b]      = 7'($urandom_range(0,127));
      tr.enq_uops_ldst[b]            = 6'($urandom_range(0,31));
      tr.enq_uops_ldst_val[b]        = 1'b1;
      tr.enq_uops_dst_rtype[b]       = 2'b01;
      tr.enq_uops_fp_val[b]          = 1'b0;
      tr.enq_uops_is_br[b]           = this_is_br;
      tr.enq_uops_br_mask[b]         = this_brmask;
      tr.enq_uops_exception[b]       = 1'b0;
      tr.enq_uops_exc_cause[b]       = 64'h0;
      tr.enq_uops_is_fence[b]        = 1'b0;
      tr.enq_uops_is_fencei[b]       = 1'b0;
      tr.enq_uops_is_unique[b]       = 1'b0;
      tr.enq_uops_flush_on_commit[b] = 1'b0;
    end
    finish_item(tr);
    @(posedge vif.clock);

    for (int b=0; b<nv; b++) begin
      bit [6:0]  gid     = (cur_tail + b) % MAX_ROB_IDX;
      bit        g_is_br = (include_new_br && b == br_bank);
      bit [19:0] g_brmask;
      bit [4:0]  g_brbit = 5'h0;

      if (g_is_br) begin
        g_brmask = current_br_mask;
        g_brbit  = new_br_bit;
      end else if (include_new_br && b > br_bank) begin
        g_brmask = current_br_mask | new_bit_mask;
      end else begin
        g_brmask = current_br_mask;
      end

      if_pdst  [gid] = tr.enq_uops_pdst[b];
      if_uopc  [gid] = tr.enq_uops_uopc[b];
      if_brmask[gid] = g_brmask;
      if_is_br [gid] = g_is_br;
      if_brbit [gid] = g_brbit;
      inflight_order.push_back(gid);
    end

    if (include_new_br) begin
      br_bit_valid  [new_br_bit] = 1'b1;
      br_bit_rob_idx[new_br_bit] = (cur_tail + br_bank) % MAX_ROB_IDX;
      current_br_mask |= new_bit_mask;
      `uvm_info("TC6_BR",
        $sformatf("Dispatch BR: bank=%0d rob_idx=%0d bit=%0d uopc=%0h => current_br_mask=%0h",
                  br_bank, br_bit_rob_idx[new_br_bit], new_br_bit,
                  tr.enq_uops_uopc[br_bank], current_br_mask),
        UVM_MEDIUM)
    end

    pred_tail_idx = wrap_inc_row_tail(cur_tail);
    ftq++;
    n_dispatch++;
  endtask

  // =======================================================================
  // WRITEBACK — FIX: thu thập eligible[] trước, chỉ remove khi thật sự WB
  // =======================================================================
  task do_random_writeback();
    rob_transaction tr;
    bit [6:0] eligible [$];
    int       nwb, cap;

    // Thu thập uop có thể WB theo shadow
    foreach (inflight_order[j]) begin
      bit [6:0] idx = inflight_order[j];
      if (drv.can_writeback(idx)) eligible.push_back(idx);
    end

    if (eligible.size() == 0) begin
      do_idle();
      return;
    end

    // Random hoá thứ tự
    eligible.shuffle();

    cap = (eligible.size() < 10) ? eligible.size() : 10;
    nwb = $urandom_range(1, cap);

    tr = rob_transaction::type_id::create("tr");
    start_item(tr);
    tr.op = ROB_WRITEBACK;
    for (int i=0; i<10; i++) tr.wb_valid[i] = 1'b0;

    for (int i=0; i<nwb; i++) begin
      bit [6:0] idx = eligible[i];
      tr.wb_valid[i]   = 1'b1;
      tr.wb_rob_idx[i] = idx;
      tr.wb_pdst[i]    = if_pdst[idx];
      remove_inflight(idx);      // chỉ remove khi THỰC SỰ WB
    end

    finish_item(tr);
    @(posedge vif.clock);
    n_wb++;
  endtask

  // =======================================================================
  // BRANCH UPDATE — 2-CYCLE PROTOCOL (giữ như v2)
  // =======================================================================
  task do_random_br_update();
    rob_transaction tr;
    int        allocated [$];
    int        pick_idx, the_bit;
    bit [6:0]  br_rid, br_uopc_snap;
    bit [19:0] br_brmask_snap, rmask, mmask;
    bit        is_misp;

    for (int i=0; i<20; i++) if (br_bit_valid[i]) allocated.push_back(i);
    if (allocated.size() == 0) begin do_idle(); return; end

    pick_idx       = $urandom_range(0, allocated.size()-1);
    the_bit        = allocated[pick_idx];
    br_rid         = br_bit_rob_idx[the_bit];
    is_misp        = ($urandom_range(1,1000) <= MIS_PROB_PERMIL);
    rmask          = 20'b1 << the_bit;
    mmask          = is_misp ? rmask : 20'h0;
    br_uopc_snap   = if_uopc.exists  (br_rid) ? if_uopc[br_rid]   : 7'h0;
    br_brmask_snap = if_brmask.exists(br_rid) ? if_brmask[br_rid] : 20'h0;

    `uvm_info("TC6_BR",
      $sformatf("BR resolve: bit=%0d rob_idx=%0d uopc=%0h orig_brmask=%0h => %s",
                the_bit, br_rid, br_uopc_snap, br_brmask_snap,
                is_misp ? "MISPREDICT" : "CORRECT"),
      UVM_MEDIUM)

    // ===== Cycle N: b1 =====
    tr = rob_transaction::type_id::create("tr");
    start_item(tr);
    tr.op                 = ROB_BRANCH_UPDATE;
    tr.br_resolve_mask    = rmask;
    tr.br_mispredict_mask = mmask;
    tr.br_rob_idx         = 7'h0;
    tr.br_mispredict      = 1'b0;
    finish_item(tr);
    @(posedge vif.clock);

    if (is_misp) kill_inflight_with_bit(the_bit);
    clear_brmask_bit(the_bit);
    br_bit_valid[the_bit] = 1'b0;
    current_br_mask      &= ~rmask;
    if (if_is_br.exists(br_rid)) if_is_br[br_rid] = 1'b0;

    if (is_misp) begin
      // ===== Cycle N+1: b2 =====
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = 20'h0;
      tr.br_mispredict_mask = 20'h0;
      tr.br_rob_idx         = br_rid;
      tr.br_mispredict      = 1'b1;
      finish_item(tr);
      @(posedge vif.clock);

      begin
        bit [6:0] br_row       = br_rid / CW;
        bit [6:0] new_tail_row = br_row + 1;
        if (new_tail_row >= NUM_ROB_ROWS) begin
          new_tail_row = 0;
          dispatch_rounds++;
        end
        pred_tail_idx = new_tail_row * CW;
      end

      repeat ($urandom_range(3,8)) do_idle();
      n_br_mispred++;
    end else begin
      n_br_correct++;
    end
  endtask

  // =======================================================================
  // IDLE
  // =======================================================================
  task do_idle();
    rob_transaction tr;
    tr = rob_transaction::type_id::create("tr");
    start_item(tr);
    tr.op = ROB_IDLE;
    finish_item(tr);
    @(posedge vif.clock);
    n_idle++;
    if (vif.mon_cb.flush_valid) begin
      `uvm_info("TC6_PTR","flush_valid -> reset toàn bộ tracking về 0", UVM_HIGH)
      flush_tracking();
    end
  endtask

  // =======================================================================
  // DRAIN — cũng áp dụng fix eligible[]
  // =======================================================================
  task drain_rob();
    rob_transaction tr;
    int att = 0;

    while (inflight_order.size() > 0 && att < 200) begin
      bit [6:0] eligible [$];
      int       nwb, cap;

      foreach (inflight_order[j]) begin
        bit [6:0] idx = inflight_order[j];
        if (drv.can_writeback(idx)) eligible.push_back(idx);
      end

      if (eligible.size() == 0) begin
        // Không có uop nào WB được lúc này, idle chờ
        tr = rob_transaction::type_id::create("tr");
        start_item(tr);
        tr.op = ROB_IDLE;
        finish_item(tr);
        @(posedge vif.clock);
        att++;
        if (vif.mon_cb.flush_valid) flush_tracking();
        continue;
      end

      cap = (eligible.size() < 10) ? eligible.size() : 10;
      nwb = cap;

      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op = ROB_WRITEBACK;
      for (int i=0; i<10; i++) tr.wb_valid[i] = 1'b0;
      for (int i=0; i<nwb; i++) begin
        bit [6:0] idx = eligible[i];
        tr.wb_valid[i]   = 1'b1;
        tr.wb_rob_idx[i] = idx;
        tr.wb_pdst[i]    = if_pdst[idx];
        remove_inflight(idx);
      end
      finish_item(tr);
      @(posedge vif.clock);
      att++;
    end

    repeat (80) begin
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op = ROB_IDLE;
      finish_item(tr);
      @(posedge vif.clock);
      sync_pointers_from_dut();
      if (vif.mon_cb.empty) begin
        `uvm_info("TC6",
          $sformatf("Drain: empty=1. Final tail=%0d head=%0d",
                    vif.mon_cb.rob_tail_idx, vif.mon_cb.rob_head_idx),
          UVM_LOW)
        return;
      end
    end
    if (!vif.mon_cb.empty) `uvm_warning("TC6","ROB not empty after drain")
  endtask

endclass
