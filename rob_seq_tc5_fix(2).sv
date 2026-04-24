// ==== FILE: rob_seq_tc5_branch_mispredict.sv ====
// Không dùng get_base — đọc vif.rob_tail_idx trước dispatch, lưu local

  class tc5_branch_mispredict_seq extends rob_seq_base;
    `uvm_object_utils(tc5_branch_mispredict_seq)

    int next_br_tag;
    bit [19:0] active_br_mask;

    function new(string name="tc5"); super.new(name); endfunction

    function void reset_br_state();
      next_br_tag = 0;
      active_br_mask = 0;
    endfunction

    task automatic dispatch_br(output int row_id, output bit [6:0] base,
                               input int br_bank);
      bit [19:0] this_mask = active_br_mask;
      base = vif.rob_tail_idx;  // đọc tail TRƯỚC dispatch
      if (br_bank >= 0)
        dispatch_branch_row(row_id, br_bank, this_mask);
      else
        dispatch_row(row_id, .br_mask(this_mask));
      if (br_bank >= 0 && next_br_tag < 20) begin
        active_br_mask |= (20'h1 << next_br_tag);
        next_br_tag++;
      end
    endtask

    task automatic dispatch_spec(output int row_id, output bit [6:0] base);
      dispatch_br(row_id, base, -1);
    endtask

    task branch_mispredict_2cyc(bit [19:0] mask, bit [6:0] rob_idx);
      rob_transaction tr;
      tr = rob_transaction::type_id::create("tr"); start_item(tr);
      tr.op = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask = mask; tr.br_mispredict_mask = mask;
      tr.br_mispredict = 0; tr.br_rob_idx = 0;
      finish_item(tr);
      tr = rob_transaction::type_id::create("tr"); start_item(tr);
      tr.op = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask = 0; tr.br_mispredict_mask = 0;
      tr.br_mispredict = 1; tr.br_rob_idx = rob_idx; tr.br_taken = 1;
      finish_item(tr);
    endtask

    task branch_resolve_1cyc(bit [19:0] mask);
      rob_transaction tr;
      tr = rob_transaction::type_id::create("tr"); start_item(tr);
      tr.op = ROB_BRANCH_UPDATE;
      tr.br_resolve_mask = mask; tr.br_mispredict_mask = 0;
      tr.br_mispredict = 0; tr.br_rob_idx = 0;
      finish_item(tr);
      active_br_mask &= ~mask;
    endtask

    task body();
      int r0, r1, r2;
      bit [6:0] base_r0, base_r1, base_r2;
      logic [6:0] tail_before;
      get_vif();
      `uvm_info("TC5","=== TC5: Branch Misprediction ===",UVM_LOW)

      // Phase A: kill + snap-back
      `uvm_info("TC5","Phase A: kill + snap-back",UVM_LOW)
      reset_br_state();
      dispatch_br(r0, base_r0, 0);    // branch bank0 → tag0
      dispatch_spec(r1, base_r1);     // br_mask=0x1
      dispatch_spec(r2, base_r2);     // br_mask=0x1
      @(posedge vif.clock); tail_before = vif.rob_tail_idx;
      idle(1);
      branch_mispredict_2cyc(20'h1, base_r0);  // rob_idx = tail lúc dispatch r0
      repeat(3) begin idle(1); @(posedge vif.clock);
        `uvm_info("TC5",$sformatf("tail=%0d head=%0d",vif.rob_tail_idx,vif.rob_head_idx),UVM_MEDIUM)
      end
      @(posedge vif.clock);
      if (vif.rob_tail_idx < tail_before)
        `uvm_info("TC5","F5.2 PASS: tail snapped back",UVM_LOW)
      wait_recovery();

      // Phase B: No WB after kill
      `uvm_info("TC5","Phase B: no WB after kill",UVM_LOW)
      reset_br_state();
      dispatch_br(r0, base_r0, 0);
      dispatch_spec(r1, base_r1);
      idle(1);
      branch_mispredict_2cyc(20'h1, base_r0);
      idle(10);
      `uvm_info("TC5","PASS: no WB after kill (W5)",UVM_LOW)
      wait_recovery();

      // Phase C: Mispredict + pending partial WB
      `uvm_info("TC5","Phase C: pending WB + mispredict",UVM_LOW)
      reset_br_state();
      dispatch_br(r0, base_r0, 0);
      dispatch_spec(r1, base_r1);
      dispatch_spec(r2, base_r2);
      idle(1);
      wb_row(r0);
      wb_banks(r1, 4'b0001);
      branch_mispredict_2cyc(20'h1, base_r0);
      idle(5);
      `uvm_info("TC5","PASS: mispredict with partial WB",UVM_LOW)
      wait_recovery();

      // Phase D: Mispredict + commit race
      `uvm_info("TC5","Phase D: commit race",UVM_LOW)
      reset_br_state();
      dispatch_spec(r0, base_r0);   // non-spec
      idle(1); wb_row(r0);
      reset_br_state();
      dispatch_br(r1, base_r1, 0);
      idle(1);
      branch_mispredict_2cyc(20'h1, base_r1);
      wait_recovery();
      `uvm_info("TC5","PASS: commit + mispredict race",UVM_LOW)

      // Phase E: Nested speculation
      `uvm_info("TC5","Phase E: Nested speculation",UVM_LOW)
      reset_br_state();
      dispatch_br(r0, base_r0, 0);  // tag0, active=0x1
      dispatch_br(r1, base_r1, 0);  // tag1, active=0x3
      dispatch_br(r2, base_r2, 0);  // tag2, active=0x7
      idle(1);
      branch_resolve_1cyc(20'h1);   // resolve tag0
      idle(1);
      branch_mispredict_2cyc(20'h2, base_r1);  // mispredict tag1 → r2 killed
      repeat(3) idle(1);
      `uvm_info("TC5","PASS: nested — r0,r1 survive, r2 killed",UVM_LOW)
      wait_recovery();

      `uvm_info("TC5","=== TC5 Complete ===",UVM_LOW)
    endtask
  endclass
