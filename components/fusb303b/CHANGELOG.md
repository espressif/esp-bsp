# ChangeLog

## v0.3.0

### Features

* Add `fusb303b_set_interrupt_mask` for the per-event Mask (0Eh) and Mask1 (0Fh) registers, with the `FUSB303B_MASK_*`/`FUSB303B_MASK1_*` bit definitions
* Add `fusb303b_read_register`/`fusb303b_write_register` raw register access for board-level policy
* Add the pure `fusb303b_decode_bc_level` helper and `FUSB303B_CURRENT_NONE`: BC_LVL=00 (Ra or nothing attached) no longer decodes as a default-current advertisement

### Enhancements

* Serialize the compound register sequences of every public entry point with a per-device mutex (read-modify-write of Control/Portrole, interrupt read-and-clear, status snapshot with clear)
* Resolve combined Portrole bits by the chip priority (DRP > sink > source, datasheet Table 14) instead of rejecting them
* Reject `scl_speed_hz` above the 400 kHz datasheet limit in `fusb303b_create`
* Cache the read-only identity registers at create time instead of re-reading them on every `fusb303b_get_status` call
* Reject a NULL handle up front in `fusb303b_get_status`, consistent with the other entry points
* Document that `fusb303b_get_and_clear_interrupts` is task-context only and that the INT_N GPIO ISR must only notify a task

## v0.1.0 - 2026-07-30

### Features

* Initial version: create/delete with identity check, enable, role and source-current selection, status snapshot, and interrupt read-and-clear
