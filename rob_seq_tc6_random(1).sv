// ============================================================================
// TC6: Constrained-Random Stimulus (REVISED v2)
//
// v1 đã có: (1) đếm action thay cycle  (2) pointer tracking wrap-aware
//
// v2 bổ sung cơ chế BRANCH UPDATE thực tế:
//   (A) is_br chỉ bật TẠI ĐÚNG bank bắt đầu branch trong row (ví dụ bank 2),
//       các bank trước nó is_br=0, br_mask giữ nguyên current_br_mask;
//       các bank SAU nó trong cùng row được OR thêm bit mới của branch.
//   (B) Branch-mask allocator cấp bit tăng dần 0 -> 19. Mỗi uop dispatch sau
//       đó mang thêm bit cho mọi branch đang in-flight. Max 20 bit in-flight;
//       bit được giải phóng khi branch tương ứng resolve.
//   (C) Branch update 2-cycle protocol:
//         cycle N   : b1_resolve_mask + b1_mispredict_mask (nếu mispredict)
//         cycle N+1 : b2_mispredict + b2_uop_rob_idx (chỉ khi mispredict)
//       Correct predict chỉ có cycle N.
//   (D) In-flight bookkeeping đầy đủ: pdst, uopc, br_mask, is_br, br_bit_pos
//       (lưu per rob_idx bằng associative arrays song song).
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

  // ---- Pointer tracking (giữ từ v1) --------------------------------------
  bit [6:0]    pred_tail_idx   = 0;
  bit [6:0]    pred_head_idx   = 0;
  int unsigned dispatch_rounds = 0;
  int unsigned commit_rounds   = 0;

  // ---- Branch-bit allocator (20 bit br_mask) -----------------------------
  bit        br_bit_valid   [20];     // bit đã cấp cho branch đang in-flight?
  bit [6:0]  br_bit_rob_idx [20];     // rob_idx của branch sở hữu bit này
  bit [19:0] current_br_mask = 20'h0; // hợp của mọi bit đang valid

  // ---- In-flight uop info (parallel assoc arrays, key = rob_idx) ---------
  bit [6:0]  if_pdst   [bit[6:0]];
  bit [6:0]  if_uopc   [bit[6:0]];
  bit [19:0] if_brmask [bit[6:0]];
  bit        if_is_br  [bit[6:0]];
  bit [4:0]  if_brbit  [bit[6:0]];    // bit position branch này sở hữu
  bit [6:0]  inflight_order [$];      // dispatch order, dùng cho WB pick

  int ftq = 0;
  int cycle_since_last_dispatch = 0;

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
    foreach (victims[i]) begin
      `uvm_info("TC6_BR",
        $sformatf("  kill speculative uop rob_idx=%0d (brmask=%0h)",
                  victims[i], if_brmask[victims[i]]), UVM_HIGH)
      remove_inflight(victims[i]);
    end
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
    rob_transaction tr;
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
    `uvm_info("TC6",
      $sformatf("Init: pred_tail=%0d pred_head=%0d", pred_tail_idx, pred_head_idx),
      UVM_LOW)

    act_done = 0;
    while (act_done < NUM_ACTIONS) begin
      // KHÔNG @clk dư ở đầu vòng — mỗi action tự xử lý clock
      action = pick_action();
      case(action)
        0: do_random_dispatch();
        1: do_random_writeback();
        2: do_random_br_update();
        3: do_idle();
      endcase
      act_done++;
      sync_pointers_from_dut();
    end

    `uvm_info("TC6",
      $sformatf("--- Drain (actions=%0d, disp_rounds=%0d, commit_rounds=%0d, live_brmask=%0h) ---",
                act_done, dispatch_rounds, commit_rounds, current_br_mask),
      UVM_LOW)
    drain_rob();
    `uvm_info("TC6","========== TC6 COMPLETE ==========",UVM_LOW)
  endtask

  // =======================================================================
  task sync_pointers_from_dut();
    bit [6:0] dut_tail = vif.mon_cb.rob_tail_idx;
    bit [6:0] dut_head = vif.mon_cb.rob_head_idx;
    if (dut_head < pred_head_idx && !vif.mon_cb.flush_valid) commit_rounds++;
    if (dut_tail !== pred_tail_idx) begin
      `uvm_info("TC6_PTR",
        $sformatf("Tail re-sync: pred=%0d -> dut=%0d", pred_tail_idx, dut_tail),
        UVM_HIGH)
      pred_tail_idx = dut_tail;
    end
    if (dut_head !== pred_head_idx) begin
      `uvm_info("TC6_PTR",
        $sformatf("Head re-sync: pred=%0d -> dut=%0d", pred_head_idx, dut_head),
        UVM_HIGH)
      pred_head_idx = dut_head;
    end
  endtask

  // =======================================================================
  function int pick_action();
    int wd=40, ww=30, wm=5, wi=25, tot, r;
    if(!vif.mon_cb.ready) begin wd=0; ww=60; wi=35; end
    if(inflight_order.size()==0) begin ww=0; wm=0; wd=70; wi=30; end
    if(current_br_mask == 20'h0) wm = 0;  // không có branch in-flight -> không resolve
    if(vif.mon_cb.empty) begin wm=0; ww=0; end
    if(vif.mon_cb.commit_rollback || vif.mon_cb.flush_valid) begin
      wd=0; ww=0; wm=0; wi=100;
    end
    if(cycle_since_last_dispatch==0) ww=0;
    tot = wd+ww+wm+wi;
    if(tot==0) return 3;
    r = $urandom_range(0, tot-1);
    if(r < wd)       return 0;
    if(r < wd+ww)    return 1;
    if(r < wd+ww+wm) return 2;
    return 3;
  endfunction

  // =======================================================================
  // DISPATCH
  //   - Quyết định có thêm branch mới vào row này không (cần bit trống).
  //   - br_bank = vị trí bank bắt đầu branch (random trong [0..nv-1]).
  //   - Bank < br_bank : is_br=0, br_mask = current_br_mask
  //     Bank = br_bank : is_br=1, br_mask = current_br_mask
  //                      (branch không speculate dưới bit của chính nó)
  //     Bank > br_bank : is_br=0, br_mask = current_br_mask | (1<<new_bit)
  //   - Sau khi @clk: update allocator + current_br_mask cho những row tới.
  // =======================================================================
  task do_random_dispatch();
    rob_transaction tr;
    bit [6:0]  cur_tail;
    int        nv;
    int        br_bank;
    int        new_br_bit;
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
          this_brmask = current_br_mask;               // branch KHÔNG mang bit của chính nó
        end else if (include_new_br && b > br_bank) begin
          this_is_br  = 1'b0;
          this_brmask = current_br_mask | new_bit_mask; // speculated dưới branch mới
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

    // Ghi tracking cho từng bank valid
    for (int b=0; b<nv; b++) begin
      bit [6:0]  gid     = (cur_tail + b) % MAX_ROB_IDX;
      bit        g_is_br = (include_new_br && b == br_bank);
      bit [19:0] g_brmask;
      bit [4:0]  g_brbit = 5'h0;

      if (g_is_br) begin
        g_brmask = current_br_mask;
        g_brbit  = new_br_bit;                        // lưu bit mà branch này sở hữu
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

    // Kích hoạt bit mới trong allocator cho các row SAU (TỪ NAY TRỞ ĐI)
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
    cycle_since_last_dispatch = 0;
  endtask

  // =======================================================================
  // WRITEBACK
  // =======================================================================
  task do_random_writeback();
    rob_transaction tr;
    int nwb, qs;

    qs = inflight_order.size();
    if (qs == 0) begin do_idle(); return; end
    nwb = $urandom_range(1, (qs<4) ? qs : 4);

    tr = rob_transaction::type_id::create("tr");
    start_item(tr);
    tr.op = ROB_WRITEBACK;
    for (int i=0; i<10; i++) tr.wb_valid[i] = 1'b0;

    for (int i=0; i<nwb; i++) begin
      int       pick;
      bit [6:0] idx;
      if (inflight_order.size() == 0) break;
      pick = $urandom_range(0, inflight_order.size()-1);
      idx  = inflight_order[pick];
      if (drv.can_writeback(idx)) begin
        tr.wb_valid[i]   = 1'b1;
        tr.wb_rob_idx[i] = idx;
        tr.wb_pdst[i]    = if_pdst[idx];
      end
      remove_inflight(idx);
    end

    finish_item(tr);
    @(posedge vif.clock);
    cycle_since_last_dispatch++;
  endtask

  // =======================================================================
  // BRANCH UPDATE — 2-CYCLE PROTOCOL
  //   Cycle N  : b1_resolve_mask = (1<<bit); b1_mispredict_mask = (mispred ? (1<<bit) : 0)
  //              b2_* = 0
  //   Cycle N+1: nếu mispredict  : b2_uop_rob_idx = br_rid; b2_mispredict = 1; b1_* = 0
  //              nếu correct     : không gửi cycle N+1
  //
  //   Sau cycle N: clear bit ra khỏi survivor; nếu mispredict kill speculative.
  //   Sau cycle N+1 (chỉ mispredict): snap pred_tail = WrapInc(row(br_rid))*CW.
  // =======================================================================
  task do_random_br_update();
    rob_transaction tr;
    int        allocated [$];
    int        pick_idx;
    int        the_bit;
    bit [6:0]  br_rid;
    bit [6:0]  br_uopc_snap;
    bit [19:0] br_brmask_snap;
    bit        is_misp;
    bit [19:0] rmask, mmask;

    // Thu thập bit đang cấp
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

    // ===== Cycle N: drive b1 =====
    tr = rob_transaction::type_id::create("tr");
    start_item(tr);
    tr.op                 = ROB_BRANCH_UPDATE;
    tr.br_resolve_mask    = rmask;
    tr.br_mispredict_mask = mmask;
    tr.br_rob_idx         = 7'h0;
    tr.br_mispredict      = 1'b0;
    finish_item(tr);
    @(posedge vif.clock);

    // --- Effect sau cycle N ---
    if (is_misp) begin
      // Kill các uop có bit the_bit bật (tức speculated dưới branch này)
      kill_inflight_with_bit(the_bit);
    end
    // Clear bit trong br_mask của các survivor
    clear_brmask_bit(the_bit);
    // Giải phóng slot allocator
    br_bit_valid[the_bit] = 1'b0;
    current_br_mask      &= ~rmask;
    // Branch uop (br_rid) chính nó: clear cờ is_br (đã resolve)
    if (if_is_br.exists(br_rid)) if_is_br[br_rid] = 1'b0;

    if (is_misp) begin
      // ===== Cycle N+1: drive b2 =====
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op                 = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask    = 20'h0;
      tr.br_mispredict_mask = 20'h0;
      tr.br_rob_idx         = br_rid;
      tr.br_mispredict      = 1'b1;
      finish_item(tr);
      @(posedge vif.clock);

      // DUT snap tail về WrapInc(row(br_rid), NUM_ROB_ROWS) * CW
      begin
        bit [6:0] br_row       = br_rid / CW;
        bit [6:0] new_tail_row = br_row + 1;
        if (new_tail_row >= NUM_ROB_ROWS) begin
          new_tail_row = 0;
          dispatch_rounds++;
        end
        pred_tail_idx = new_tail_row * CW;
      end

      // Cool-down idle để DUT ổn định sau snap
      repeat ($urandom_range(3,8)) do_idle();
    end

    cycle_since_last_dispatch++;
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
    cycle_since_last_dispatch++;
    if (vif.mon_cb.flush_valid) begin
      `uvm_info("TC6_PTR","flush_valid -> reset toàn bộ tracking về 0", UVM_HIGH)
      flush_tracking();
    end
  endtask

  // =======================================================================
  // DRAIN
  // =======================================================================
  task drain_rob();
    rob_transaction tr;
    int att = 0;

    while (inflight_order.size() > 0 && att < 100) begin
      int nwb = (inflight_order.size() < 10) ? inflight_order.size() : 10;
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op = ROB_WRITEBACK;
      for (int i=0; i<10; i++) tr.wb_valid[i] = 1'b0;
      for (int i=0; i<nwb && inflight_order.size()>0; i++) begin
        bit [6:0] idx = inflight_order[0];
        if (drv.can_writeback(idx)) begin
          tr.wb_valid[i]   = 1'b1;
          tr.wb_rob_idx[i] = idx;
          tr.wb_pdst[i]    = if_pdst[idx];
        end
        remove_inflight(idx);
      end
      finish_item(tr);
      @(posedge vif.clock);
      att++;
    end

    repeat (60) begin
      tr = rob_transaction::type_id::create("tr");
      start_item(tr);
      tr.op = ROB_IDLE;
      finish_item(tr);
      @(posedge vif.clock);
      sync_pointers_from_dut();
      if (vif.mon_cb.empty) begin
        `uvm_info("TC6",
          $sformatf("Drain: empty=1. Final tail=%0d head=%0d (!=0 là HỢP LỆ)",
                    vif.mon_cb.rob_tail_idx, vif.mon_cb.rob_head_idx),
          UVM_LOW)
        return;
      end
    end
    if (!vif.mon_cb.empty) `uvm_warning("TC6","ROB not empty after drain")
  endtask

endclass
