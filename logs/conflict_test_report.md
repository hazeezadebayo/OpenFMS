# OpenFMS Fleet Manager — Conflict Test Report
> **Generated:** 2026-02-22 20:13:29
> **Result: 10/16 passed** | 6 failed

> All tests call **real** `FmTrafficHandler` / `FmScheduleHandler` methods.
> Only psycopg2, skfuzzy, and the 6 DB/MQTT submodule classes are mocked.

---

## Summary Table

| # | Scenario | Status | FM Decision | Code Path |
|---|----------|--------|-------------|-----------|
| — | **S1: Same Target (C10)** | ❌ |  | `_handle_no_conflict_case (R01) / _handle_active_mex_conflict (R02)` |
| — | **S2a: No-swap – A@C17(W17,high) vs B@C29(W29,low). B yields at W29.** | ✅ | B: lower priority case. | `handle_priority_lower:1877 → B wait at W29 / A mex_wait_time=None → update_robot` |
| — | **S2b: No-swap – A@C10(high,no WP) vs B@C2(low,no WP). Exchange via NEIGHBOUR YIELD.** | ✅ | NEIGHBOR YIELD (exchange): MEX yields to C18 (spoofed W18). B proceeds to C10. | `handle_priority_lower:1860 → handle_priority_higher(from_source=False):1791 → NE` |
| — | **S2c: No-swap – A@C2(high,no WP) vs B@C1(low,no WP). C10 free → A yields to C10 (exchange).** | ✅ | NEIGHBOR YIELD (exchange): MEX yields to C10 (spoofed W10). B proceeds to C2. | `handle_priority_lower:1860 → handle_priority_higher(from_source=False) → NEIGHBO` |
| — | **S2d: No-swap – all C2 neighbours occupied → human assist.** | ✅ | MEX mex --> could not find waitpoint in graph. human help required. | `handle_priority_higher:1791-1797 (no free neighbour → error log)` |
| — | **S3a: Swap – A@C17(W17,high)↔B@C9(low). B yields at WP.** | ✅ | A: high priority case. | `_handle_active_mex_conflict:1459 → handle_priority_higher (A wins) → B sent to w` |
| — | **S3b: Swap – A@C17(W17,high)↔B@C9(no WP,low). Neighbour yield for B.** | ✅ | A: high priority case. | `handle_priority_higher:1759 → NEIGHBOR YIELD PATCH (B has no WP)` |
| — | **S3c: Swap – A@C10(no WP)↔B@C18(no WP). Both have free neighbours.** | ✅ | NEIGHBOR YIELD: MEX yielding to neighbor C26 (spoofed as W26) | `handle_priority_higher:1760 → NEIGHBOR YIELD PATCH` |
| — | **S3d: Swap – A@C2(no WP)↔B@C3(no WP). All neighbours occupied.** | ✅ | A --> could not find waitpoint in graph. human help required. | `handle_priority_higher:1791 (from_source=False) → error log, human assist` |
| — | **S4: Post-resolution continuation (no freeze)** | ❌ |  | `_handle_no_conflict_case → fm_file_task → build_order_msg` |
| — | **S5a: Mutex [C10,C11] – R02@C11 blocks R01 from C10** | ✅ | Added Mutex Group: ['C10', 'C11'] | `_handle_robot_traffic_status:446-453 (mutex expand) → C10 in expanded_traffic → ` |
| — | **S5b: Mutex [C10,C11] inactive – no member in traffic, R01 approved for C10** | ❌ | C10 is free → _handle_no_conflict_case | `_handle_no_conflict_case → fm_file_task → build_order_msg` |
| — | **S6: Task to busy robot – FM re-routes or queues** | ❌ |  | `fm_send_task_request:260-288 → _queue_for_robot (S6b) or reassign (S6a)` |
| — | **S7a: Low battery (15%) → auto-charge task filed** | ❌ | Robot R01: battery level lower than minimum required. | `_handle_low_battery:454 → _move_to_dock → fm_send_task_request(task=charge)` |
| — | **S7b: Low battery, all charge docks occupied → no order** | ✅ | _handle_low_battery returned False. No charge order filed. | `_handle_low_battery:458 → no free dock found → return False` |
| — | **REGRESSION: Waitpoint 2-CP order – no freeze after fix** | ❌ | R01 wait case completed scenario. green. | `_handle_waitpoint_case → create_order (FmTrafficHandler.py:678)` |

---

## Detailed Results

### ❌ S1: Same Target (C10)

| Field | Value |
|-------|-------|
| **Robots** | R01@C2, R02@C9. Both target C10. |
| **C10 neighbours** | ['C11', 'C18', 'C2', 'C9'] |
| **R01 result (C10 free)** | build_order_msg called: False. checkpoints in spy: [] |
| **R02 result (C10 taken)** | build_order_msg called: False (expected False) |
| **R02 conflict logs** | Robot R02: next_stop_id C10 occupied.; R02 waiting for mex_r_id MEX to pass. no possible conflict expected. |
| **Code path** | _handle_no_conflict_case (R01) / _handle_active_mex_conflict (R02) |
| **Evaluation** | FAIL |

### ✅ S2a: No-swap – A@C17(W17,high) vs B@C29(W29,low). B yields at W29.

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C17, waitpoint=W17 (A docks here — not a conflict hold) |
| **B (low prio)** | node=C29, waitpoint=W29, target=C17 |
| **Traffic** | [C17, C29] |
| **FM Decision** | B: lower priority case. |
| **Order 1 → B (r_id)** | checkpoints=['C29', 'C17'] waitpoints=['W29'] wait_time='49.47726750741192' → WAITS |
| **Order 2 → A (mex_r_id)** | checkpoints=['C17', 'W17'] waitpoints=['W17'] wait_time=None → PROCEEDS |
| **CRITICAL: B wait_time** | 49.47726750741192 seconds (B holds) |
| **CRITICAL: A wait_time** | None (None=A proceeds NOW) |
| **B waits, A proceeds?** | B_waits=True, A_proceeds=True |
| **Key FM logs** | B: lower priority case.; currently reserved: C29. temp_fb_wait_traffic: W29.; B --> wait at W29. |
| **Code path** | handle_priority_lower:1877 → B wait at W29 / A mex_wait_time=None → update_robot_status |
| **Evaluation** | PASS: B waits at W29, A proceeds unblocked |

### ✅ S2b: No-swap – A@C10(high,no WP) vs B@C2(low,no WP). Exchange via NEIGHBOUR YIELD.

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C10, no waitpoint |
| **B (low prio)** | node=C2, no waitpoint, target=C10 |
| **Traffic** | [C10, C2] |
| **C10 free neighbours (A can yield to)** | ['C11', 'C18', 'C9'] |
| **FM path** | B low-prio → handle_priority_lower → handle_priority_higher(from_source=False) → NEIGHBOUR YIELD EXCHANGE |
| **Exchange logic** | A yields to free C10-neighbour (spoof WP). B proceeds to C10. Cx stays in traffic while A waits. |
| **FM Decision** | NEIGHBOR YIELD (exchange): MEX yields to C18 (spoofed W18). B proceeds to C10. |
| **Orders published** | 2 order(s) (2 expected: one per robot). WPs per order: [[], ['W18']]. |
| **Key FM logs** | NEIGHBOR YIELD (exchange): MEX yields to C18 (spoofed W18). B proceeds to C10. |
| **Code path** | handle_priority_lower:1860 → handle_priority_higher(from_source=False):1791 → NEIGHBOUR YIELD EXCHANGE |
| **Evaluation** | PASS: exchange triggered, both robots issued orders |

### ✅ S2c: No-swap – A@C2(high,no WP) vs B@C1(low,no WP). C10 free → A yields to C10 (exchange).

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C2, no waitpoint |
| **B (low prio)** | node=C1, no waitpoint, target=C2 |
| **Traffic** | [C2, C1, C3] |
| **C2 neighbours** | ['C1', 'C10', 'C3'] |
| **C2 free neighbours (A can yield to)** | ['C10'] |
| **FM path** | B low-prio → handle_priority_lower → handle_priority_higher(from_source=False) → NEIGHBOUR YIELD EXCHANGE |
| **Exchange logic** | A yields to C10 (spoof W10). B gets C2. Cx=C1 stays in traffic (B is heading there after). |
| **FM Decision** | NEIGHBOR YIELD (exchange): MEX yields to C10 (spoofed W10). B proceeds to C2. |
| **Orders published** | 2 order(s). WPs: [[], ['W10']]. |
| **Key FM logs** | NEIGHBOR YIELD (exchange): MEX yields to C10 (spoofed W10). B proceeds to C2. |
| **Code path** | handle_priority_lower:1860 → handle_priority_higher(from_source=False) → NEIGHBOUR YIELD EXCHANGE (C10) |
| **Evaluation** | PASS: exchange via C10 triggered |

### ✅ S2d: No-swap – all C2 neighbours occupied → human assist.

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C2, no waitpoint |
| **B (low prio)** | node=C1, no waitpoint, target=C2 |
| **Traffic** | [C2, C1, C3, C10] |
| **C2 neighbours all occupied** | C1(B), C3(traffic), C10(traffic) |
| **FM Decision** | MEX mex --> could not find waitpoint in graph. human help required. |
| **Orders published** | 0 (expected 0) |
| **Key FM logs** | B: could not find a node associated waitpoint. will try for high priority.; MEX mex --> could not find waitpoint in graph. human help required.; B and mex_r_id MEX stuck because waitpoint was not found in graph and no re-route/alternative path. Human assistance required. |
| **Code path** | handle_priority_higher:1791-1797 (no free neighbour → error log) |
| **Evaluation** | PASS: human assist logged |

### ✅ S3a: Swap – A@C17(W17,high)↔B@C9(low). B yields at WP.

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C17, waitpoint=W17, target=C9 |
| **B (low prio)** | node=C9, no waitpoint, target=C17 |
| **Traffic** | [C17, C9] |
| **Swap detected** | A→C9, B→C17 |
| **FM Decision** | A: high priority case. |
| **Orders published** | 2 order(s). All WPs: [['W17'], ['W10']] |
| **Key FM logs** | A: high priority case.; MEX mex --> wait at W10. |
| **Code path** | _handle_active_mex_conflict:1459 → handle_priority_higher (A wins) → B sent to wait |
| **Evaluation** | PASS: priority-based yield triggered |

### ✅ S3b: Swap – A@C17(W17,high)↔B@C9(no WP,low). Neighbour yield for B.

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C17, waitpoint=W17, target=C9 |
| **B (low prio)** | node=C9, no waitpoint, target=C17 |
| **Traffic** | [C17, C9] |
| **B neighbours (free)** | ['C10'] |
| **FM Decision** | A: high priority case. |
| **Orders published** | 2 order(s). WPs: [['W17'], ['W10']] |
| **Key FM logs** | A: high priority case.; NEIGHBOR YIELD: MEX yielding to neighbor C10 (spoofed as W10) |
| **Code path** | handle_priority_higher:1759 → NEIGHBOR YIELD PATCH (B has no WP) |
| **Evaluation** | PASS: neighbour yield / priority yield triggered |

### ✅ S3c: Swap – A@C10(no WP)↔B@C18(no WP). Both have free neighbours.

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C10, no waitpoint, target=C18 |
| **B (low prio)** | node=C18, no waitpoint, target=C10 |
| **Traffic** | [C10, C18] |
| **C18 free neighbours** | ['C17', 'C19', 'C26'] |
| **C10 free neighbours** | ['C11', 'C2', 'C9'] |
| **FM Decision** | NEIGHBOR YIELD: MEX yielding to neighbor C26 (spoofed as W26) |
| **Orders published** | 2 order(s). WPs: [[], ['W26']] |
| **Key FM logs** | NEIGHBOR YIELD: MEX yielding to neighbor C26 (spoofed as W26) |
| **Code path** | handle_priority_higher:1760 → NEIGHBOR YIELD PATCH |
| **Evaluation** | PASS: spoof WP created from free neighbour |

### ✅ S3d: Swap – A@C2(no WP)↔B@C3(no WP). All neighbours occupied.

| Field | Value |
|-------|-------|
| **A (high prio)** | node=C2, no waitpoint, target=C3 |
| **B (low prio)** | node=C3, no waitpoint, target=C2 |
| **Traffic** | [C2, C3, C1, C10, C4, C11] |
| **C3 neighbours** | ['C11', 'C2', 'C4'] |
| **C2 neighbours** | ['C1', 'C10', 'C3'] |
| **FM Decision** | A --> could not find waitpoint in graph. human help required. |
| **Orders published** | 0 (expected 0) |
| **Key FM logs** | A --> could not find waitpoint in graph. human help required.; A and mex_r_id MEX stuck because waitpoint was not found in graph and no re-route/alternative path. Human assistance required. |
| **Code path** | handle_priority_higher:1791 (from_source=False) → error log, human assist |
| **Evaluation** | PASS: human assist required logged |

### ❌ S4: Post-resolution continuation (no freeze)

| Field | Value |
|-------|-------|
| **Purpose** | After each S3 conflict is resolved, confirm A can publish next goal (no freeze) |
| **S3a resolved (A@C17→C9, W17)** | order_fired=False, checkpoints=[], freeze=True |
| **S3b resolved (A@C17→C9, W17)** | order_fired=False, checkpoints=[], freeze=True |
| **S3c resolved (A@C10→C18)** | order_fired=False, checkpoints=[], freeze=True |
| **S3d resolved (A@C2→C3)** | order_fired=False, checkpoints=[], freeze=True |
| **Code path** | _handle_no_conflict_case → fm_file_task → build_order_msg |
| **Evaluation** | FAIL: some continuations did not publish an order |

### ✅ S5a: Mutex [C10,C11] – R02@C11 blocks R01 from C10

| Field | Value |
|-------|-------|
| **R01** | node=C2, target=C10 |
| **R02** | node=C11 (traffic) |
| **Mutex group** | [C10, C11] |
| **Traffic (raw)** | [C11] |
| **After mutex expansion** | [C11, C10] |
| **FM Decision** | Added Mutex Group: ['C10', 'C11'] |
| **Orders published** | 0 (expected 0 — C10 mutex-blocked) |
| **Code path** | _handle_robot_traffic_status:446-453 (mutex expand) → C10 in expanded_traffic → conflict |
| **Evaluation** | PASS: R01 blocked from C10 via mutex |

### ❌ S5b: Mutex [C10,C11] inactive – no member in traffic, R01 approved for C10

| Field | Value |
|-------|-------|
| **R01** | node=C2, target=C10 |
| **Traffic** | [C17] — neither C10 nor C11 |
| **Mutex group** | [C10, C11] |
| **After mutex expansion** | [C17] — no mutex member in traffic, nothing added |
| **FM Decision** | C10 is free → _handle_no_conflict_case |
| **Orders published** | 0 order(s). checkpoints=[] |
| **Code path** | _handle_no_conflict_case → fm_file_task → build_order_msg |
| **Evaluation** | FAIL |

### ❌ S6: Task to busy robot – FM re-routes or queues

| Field | Value |
|-------|-------|
| **S6a Robots** | R01@C20(idle,home), R02@C10(busy,node_states=[C11]) |
| **S6a Request** | Transport C1→C5 for R02 |
| **S6a FM Decision** | chosen_robot=R01. cleared=True |
| **S6a Reassign log** | User robot R02 busy. Reassigning to next best: R01 (fitness: 0.800) |
| **S6a Orders** | 0 order(s), robot=?, checkpoints=[] |
| **S6b Robots** | Only R02@C10(busy) |
| **S6b FM Decision** | cleared=False (False=queued) |
| **S6b Unassigned filed** | False |
| **S6b Orders robot field** | [] |
| **Code path** | fm_send_task_request:260-288 → _queue_for_robot (S6b) or reassign (S6a) |
| **Evaluation** | FAIL: chosen_a=R01, cleared_b=False, unassigned_filed=False |

### ❌ S7a: Low battery (15%) → auto-charge task filed

| Field | Value |
|-------|-------|
| **R01** | node=C20 (home_dock), battery=15% |
| **min_charge_level** | 33% |
| **Charge docks** | ['C23', 'C50'] |
| **Traffic (docks)** | [C20] — charge docks free |
| **FM Decision** | Robot R01: battery level lower than minimum required. |
| **_handle_low_battery returned** | False |
| **Orders published** | 0 order(s). checkpoints=[] |
| **Code path** | _handle_low_battery:454 → _move_to_dock → fm_send_task_request(task=charge) |
| **Evaluation** | FAIL |

### ✅ S7b: Low battery, all charge docks occupied → no order

| Field | Value |
|-------|-------|
| **R01** | node=C20 (home_dock), battery=15% |
| **Traffic** | ['C23', 'C50', 'C20'] — ALL charge docks occupied |
| **FM Decision** | _handle_low_battery returned False. No charge order filed. |
| **Orders published** | 0 (expected 0) |
| **Code path** | _handle_low_battery:458 → no free dock found → return False |
| **Evaluation** | PASS: correctly skipped when all docks busy |

### ❌ REGRESSION: Waitpoint 2-CP order – no freeze after fix

| Field | Value |
|-------|-------|
| **R01** | node=C17, waitpoint=W17, WP-order checkpoints=[C17,C18] |
| **merged_nodes[0]** | W17 (triggers _handle_waitpoint_case) |
| **Pre-fix behaviour** | horizon was checkps[2:] → empty → no order → FREEZE |
| **Post-fix behaviour** | horizon set to checkps[1:] for 2-CP orders → [C18] → order issued |
| **FM Decision** | R01 wait case completed scenario. green. |
| **Orders published** | 0 order(s). checkpoints=[] |
| **Code path** | _handle_waitpoint_case → create_order (FmTrafficHandler.py:678) |
| **Evaluation** | FAIL: FREEZE detected |

---

## FM Decision Logic Reference

| Decision | Trigger | Code Location |
|----------|---------|---------------|
| Reserve node (no conflict) | `next_stop_id` not in `traffic_control` | `_handle_no_conflict_case` |
| Conflict detected | `next_stop_id` in `traffic_control` | `_handle_active_mex_conflict` |
| Priority-higher wins | `priority_val > mex_priority_val` | `handle_priority_higher:1459` |
| Priority-lower yields at WP | own WP found for `reserved_checkpoint` | `handle_priority_lower:1842` |
| Neighbour Yield Patch | no WP found, free graph neighbour exists | `handle_priority_higher:1760-1783` |
| Human assist | no WP and no free neighbour | `handle_priority_higher:1791-1797` |
| Mutex expansion | mutex group member in traffic | `_handle_robot_traffic_status:446-453` |
| Queue unassigned | robot busy, no free robot | `_queue_for_robot:407-417` |
| Auto-reassign | user robot busy, free robot exists | `fm_send_task_request:268-278` |
| Charge task | battery < min_charge_level | `_handle_low_battery:454` |
| Waitpoint re-order | `merged_nodes[0]` starts with 'W' | `_handle_waitpoint_case:678` |
