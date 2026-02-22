
[1m[36m========================================================================[0m
[1m[36mS1: Same Target Conflict (R01 and R02 want C10)[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot R01:[0m base=C2, order -> checkpoints=[C2, C10, C11, C3] (Released up to C2)
[1mRobot R02:[0m base=C9, order -> checkpoints=[C9, C10, C2] (Released up to C9)
[1mTraffic Constraint:[0m [] (Initially free)
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for R01...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C2', 'C9'].
   [36m[FM DECISION][0m 
R_id: R01. 
Reserved_checkpoint: C2, Next_stop_id: C10, Status: red  
Horizon: ['C10', 'C11', 'C3'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m R01 no conflict reservation. green.
   [32m[ORDER PUBLISHED][0m Robot: R01 | Wait Time: None
                     Checkpoints: ['C2', 'C10', 'C11', 'C3']
                     Waitpoints:  []
                     Horizon:     ['C10', 'C11', 'C3']


[36m--- R01 is now en route, holding traffic reservation on C10 ---[0m

[32m[Execute] Analysing traffic state for R02...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C2', 'C10', 'C9'].
   [36m[FM DECISION][0m 
R_id: R02. 
Reserved_checkpoint: C9, Next_stop_id: C10, Status: red  
Horizon: ['C10', 'C2'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot R02: next_stop_id C10 occupied.
   [36m[FM DECISION][0m mex_r_id 'version' could not be verified. handshake incomplete.

[1m[36m========================================================================[0m
[1m[36mS2a: No-swap Conflict [A & B nodes have waitpoints] – A(MEX)@C17(W17) vs B(R_ID)@C29(W29).[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C17, pending order to move to W17
[1mRobot B (Low Prio):[0m base=C29, heading towards C17
[1mTraffic Constraint:[0m [C17, W17, C29]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for B...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C17', 'W17', 'C29'].
   [36m[FM DECISION][0m 
R_id: B. 
Reserved_checkpoint: C29, Next_stop_id: C17, Status: red  
Horizon: ['C17', 'C18'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot B: next_stop_id C17 occupied.
   [36m[FM DECISION][0m B  waiting for mex_r_id A to reach decision node.

[1m[36m========================================================================[0m
[1m[36mS2b: No-swap Conflict [No waitpoints, both have free C_nodes] – A@C18 vs B@C28.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C18, holding. Connected free node C19.
[1mRobot B (Low Prio):[0m base=C28, wants C18. Connected free node C20.
[1mTraffic Constraint:[0m [C18, C28]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for B...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C18', 'C28'].
   [36m[FM DECISION][0m 
R_id: B. 
Reserved_checkpoint: C28, Next_stop_id: C18, Status: red  
Horizon: ['C18'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot B: next_stop_id C18 occupied.
   [36m[FM DECISION][0m B waiting for mex_r_id A to pass. no possible conflict expected.

[1m[36m========================================================================[0m
[1m[36mS2c: No-swap Conflict [Only one has free C_node] – A@C20 vs B@C21.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C20, completely blocked.
[1mRobot B (Low Prio):[0m base=C21(has free neighbor), wants C20.
[1mTraffic Constraint:[0m [C20, C21]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for B...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C20', 'C21'].
   [36m[FM DECISION][0m 
R_id: B. 
Reserved_checkpoint: C21, Next_stop_id: C20, Status: red  
Horizon: ['C20'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot B: next_stop_id C20 occupied.
   [36m[FM DECISION][0m B  waiting for mex_r_id A to reach decision node.

[1m[36m========================================================================[0m
[1m[36mS2d: No-swap Conflict [No waitpoints, no free C_nodes] – A@C6 vs B@C7.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C6, deadlocked environment.
[1mRobot B (Low Prio):[0m base=C7, wants C6, deadlocked environment.
[1mTraffic Constraint:[0m [C6, C7]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for B...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C6', 'C7'].
   [36m[FM DECISION][0m 
R_id: B. 
Reserved_checkpoint: C7, Next_stop_id: C6, Status: red  
Horizon: ['C6'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot B: next_stop_id C6 occupied.
   [36m[FM DECISION][0m B  waiting for mex_r_id A to reach decision node.

[1m[36m========================================================================[0m
[1m[36mS3a: Swap Conflict [Both have waitpoints] – A@C17(W17) wants C12 ↔ B@C12(W12) wants C17.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C17, wants C12. Has W17.
[1mRobot B (Low Prio):[0m base=C12, wants C17. Has W12.
[1mTraffic Constraint:[0m [C17, C12]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for A...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C17', 'W17', 'C12', 'W12'].
   [36m[FM DECISION][0m 
R_id: A. 
Reserved_checkpoint: C17, Next_stop_id: C12, Status: red  
Horizon: ['C12'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot A: next_stop_id C12 occupied.
   [36m[FM DECISION][0m A started negotiation.
   [36m[FM DECISION][0m A: high priority case.
   [36m[FM DECISION][0m Checking paths --> [[]].
   [36m[FM DECISION][0m Reroute_status --> False.
   [36m[FM DECISION][0m Reroute_status --> False.
   [36m[FM DECISION][0m Fleet kullar <-> A will try to find reroute path to pass.
   [36m[FM DECISION][0m A and mex_r_id B stuck because waitpoint was not found in graph and no re-route/alternative path. Human assistance required.

[1m[36m========================================================================[0m
[1m[36mS3b: Swap Conflict [Only one has waitpoint] – A(R_ID, high)@C17 wants C9 ↔ B(MEX, low)@C9 wants C17.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A:[0m base=C17, wants C9. Has W17.
[1mRobot B:[0m base=C9, wants C17. No waitpoint.
[1mTraffic Constraint:[0m [C17, C9]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for A...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C17', 'W17', 'C9'].
   [36m[FM DECISION][0m 
R_id: A. 
Reserved_checkpoint: C17, Next_stop_id: C9, Status: red  
Horizon: ['C9', 'C10'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot A: next_stop_id C9 occupied.
   [36m[FM DECISION][0m A started negotiation.
   [36m[FM DECISION][0m A: high priority case.
   [36m[FM DECISION][0m NEIGHBOR YIELD: B yielding to neighbor C10 (spoofed as W10)
   [36m[FM DECISION][0m B mex --> wait at W10.
   [36m[FM DECISION][0m B mex --> estimated wait time 12.0.
   [36m[FM DECISION][0m r_id: A --> estimated wait time None.
   [36m[FM DECISION][0m mex_r_id: B --> mex estimated wait time 12.0.
   [36m[FM DECISION][0m mex_r_id: B is in an elevator but can not fetch new map.
   [36m[FM DECISION][0m r_id: A completed negotiation.
   [32m[ORDER PUBLISHED][0m Robot: A | Wait Time: None
                     Checkpoints: ['C17', 'C9', 'C10']
                     Waitpoints:  ['W17']
                     Horizon:     ['C9', 'C10']

   [32m[ORDER PUBLISHED][0m Robot: B | Wait Time: 12.0
                     Checkpoints: ['C9', 'C17', 'C18']
                     Waitpoints:  ['W10']
                     Horizon:     ['C17', 'C18']


[1m[36m========================================================================[0m
[1m[36mS3c: Swap Conflict [No waitpoints, both have free C_nodes] – A@C20 wants C21 ↔ B@C21 wants C20.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C20, wants C21. Has free C19.
[1mRobot B (Low Prio):[0m base=C21, wants C20. Has free C22.
[1mTraffic Constraint:[0m [C20, C21]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for A...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C20', 'C21'].
   [36m[FM DECISION][0m 
R_id: A. 
Reserved_checkpoint: C20, Next_stop_id: C21, Status: red  
Horizon: ['C21'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot A: next_stop_id C21 occupied.
   [36m[FM DECISION][0m A started negotiation.
   [36m[FM DECISION][0m A: high priority case.
   [36m[FM DECISION][0m NEIGHBOR YIELD: B yielding to neighbor C13 (spoofed as W13)
   [36m[FM DECISION][0m B mex --> wait at W13.
   [36m[FM DECISION][0m B mex --> estimated wait time 12.0.
   [36m[FM DECISION][0m r_id: A --> estimated wait time None.
   [36m[FM DECISION][0m mex_r_id: B --> mex estimated wait time 12.0.
   [36m[FM DECISION][0m mex_r_id: B is in an elevator but can not fetch new map.
   [36m[FM DECISION][0m r_id: A completed negotiation.
   [32m[ORDER PUBLISHED][0m Robot: A | Wait Time: None
                     Checkpoints: ['C20', 'C21']
                     Waitpoints:  []
                     Horizon:     ['C21']

   [32m[ORDER PUBLISHED][0m Robot: B | Wait Time: 12.0
                     Checkpoints: ['C21', 'C20']
                     Waitpoints:  ['W13']
                     Horizon:     ['C20']


[1m[36m========================================================================[0m
[1m[36mS3d: Swap Conflict [No waitpoints, only one has free C_node] – A@C22 wants C23 ↔ B@C23 wants C22.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C22, wants C23. Blocked behind it.
[1mRobot B (Low Prio):[0m base=C23, wants C22. Has free C24.
[1mTraffic Constraint:[0m [C22, C23]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for A...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C22', 'C23'].
   [36m[FM DECISION][0m 
R_id: A. 
Reserved_checkpoint: C22, Next_stop_id: C23, Status: red  
Horizon: ['C23'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot A: next_stop_id C23 occupied.
   [36m[FM DECISION][0m A started negotiation.
   [36m[FM DECISION][0m A: high priority case.
   [36m[FM DECISION][0m NEIGHBOR YIELD: B yielding to neighbor C31 (spoofed as W31)
   [36m[FM DECISION][0m B mex --> wait at W31.
   [36m[FM DECISION][0m B mex --> estimated wait time 12.0.
   [36m[FM DECISION][0m r_id: A --> estimated wait time None.
   [36m[FM DECISION][0m mex_r_id: B --> mex estimated wait time 12.0.
   [36m[FM DECISION][0m mex_r_id: B is in an elevator but can not fetch new map.
   [36m[FM DECISION][0m r_id: A completed negotiation.
   [32m[ORDER PUBLISHED][0m Robot: A | Wait Time: None
                     Checkpoints: ['C22', 'C23']
                     Waitpoints:  []
                     Horizon:     ['C23']

   [32m[ORDER PUBLISHED][0m Robot: B | Wait Time: 12.0
                     Checkpoints: ['C23', 'C22']
                     Waitpoints:  ['W31']
                     Horizon:     ['C22']


[1m[36m========================================================================[0m
[1m[36mS4: Post-Resolution Continuation – A(High)@C17 wants [C9,C10,C11] ↔ B(Low)@C9 wants [C17,C18,C19].[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C17, wants C9 then C10. Has W17.
[1mRobot B (Low Prio):[0m base=C9, wants C17 then C18. No waitpoint.
[1mTraffic Constraint:[0m [C17, C9]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for A...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C17', 'W17', 'C9'].
   [36m[FM DECISION][0m 
R_id: A. 
Reserved_checkpoint: C17, Next_stop_id: C9, Status: red  
Horizon: ['C9', 'C10', 'C11'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot A: next_stop_id C9 occupied.
   [36m[FM DECISION][0m A started negotiation.
   [36m[FM DECISION][0m A: high priority case.
   [36m[FM DECISION][0m NEIGHBOR YIELD: B yielding to neighbor C10 (spoofed as W10)
   [36m[FM DECISION][0m B mex --> wait at W10.
   [36m[FM DECISION][0m B mex --> estimated wait time 12.0.
   [36m[FM DECISION][0m r_id: A --> estimated wait time None.
   [36m[FM DECISION][0m mex_r_id: B --> mex estimated wait time 12.0.
   [36m[FM DECISION][0m mex_r_id: B is in an elevator but can not fetch new map.
   [36m[FM DECISION][0m r_id: A completed negotiation.
   [32m[ORDER PUBLISHED][0m Robot: A | Wait Time: None
                     Checkpoints: ['C17', 'C9', 'C10', 'C11']
                     Waitpoints:  ['W17']
                     Horizon:     ['C9', 'C10', 'C11']

   [32m[ORDER PUBLISHED][0m Robot: B | Wait Time: 12.0
                     Checkpoints: ['C9', 'C17', 'C18', 'C19']
                     Waitpoints:  ['W10']
                     Horizon:     ['C17', 'C18', 'C19']


[36m--- Advancing State: B yields to W10. A advances to C9 ---[0m

[32m[Execute] Analysing traffic state for A...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C17', 'C9', 'W17', 'W10'].
   [36m[FM DECISION][0m 
R_id: A. 
Reserved_checkpoint: C9, Next_stop_id: C9, Status: red  
Horizon: ['C9', 'C10', 'C11'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: True, second cond.: False.

[32m[Execute] Analysing traffic state for B...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C17', 'C9', 'W17', 'W10'].
   [36m[FM DECISION][0m 
R_id: B. 
Reserved_checkpoint: W10, Next_stop_id: C17, Status: red  
Horizon: ['C17', 'C18', 'C19'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: True, second cond.: False.
   [36m[FM DECISION][0m Robot B: next_stop_id C17 occupied.
   [36m[FM DECISION][0m mex_r_id 'version' could not be verified. handshake incomplete.

[1m[36m========================================================================[0m
[1m[36mS5: Mutex Group Conflict – A@C2 (Inside Mutex group [C2,C3]) ↔ B@C4 wants C3.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C2 (which is mutex-coupled with C3).
[1mRobot B (Low Prio):[0m base=C4, wants C3.
[1mTraffic Constraint:[0m [C2] + Mutex[C2, C3]
[1m-----------------------------------[0m

[32m[Execute] Analysing traffic state for B...[0m
 
   [36m[FM DECISION][0m Traffic Control: ['C2', 'C4'].
   [36m[FM DECISION][0m 
R_id: B. 
Reserved_checkpoint: C4, Next_stop_id: C3, Status: red  
Horizon: ['C3'], 
Dist_to_base: 0.0, has_order_minute_passed: True, temp_fb_dock_action_done: True, first cond.: False, second cond.: True.
   [36m[FM DECISION][0m Robot B: next_stop_id C3 occupied.
   [36m[FM DECISION][0m mex_r_id 'version' could not be verified. handshake incomplete.

[1m[36m========================================================================[0m
[1m[36mS6: Task Queueing – Assign task while robot A is busy.[0m
[1m[36m========================================================================[0m
[1m--- NATIVE STATE REPRESENTATION ---[0m
[1mRobot A (High Prio):[0m base=C2, actively holding an order to C10.
[1mTraffic Constraint:[0m N/A - Evaluating Task Assignment while busy
[1m-----------------------------------[0m

[36m[Execute] Testing task assignment on occupied robot...[0m
   [36m[FM DECISION][0m A: connection state is ONLINE.
   [36m[FM DECISION][0m robot position: 0.0, 0.0, 0.0.
   [36m[FM DECISION][0m Found shortest distance: 0.0 with loc: C1.
   [36m[FM DECISION][0m Robot A: last node id not a home dock.
   [36m[FM DECISION][0m Robot A: previous task was not cancelled and robot is not at a home dock.
      [36mReturned Available Robots from assignment parser: None[0m

[1m[36mNative Database Simulation Completed.[0m


========================================================================
S7: Low Battery Trigger
========================================================================

--- NATIVE STATE REPRESENTATION ---
Robot A (Idle): base=C2, idle, battery=15%.
Traffic Constraint: N/A - Evaluating Automatic Charge Dispatch
-----------------------------------

[Execute] Testing manage_robot on low-battery idle robot...
   [FM DECISION] A: connection state is ONLINE.
   [FM DECISION] robot position: 0.0, 0.0, 0.0.
   [FM DECISION] Found shortest distance: 0.0 with loc: C1.
   [FM DECISION] Robot A: last node id not a home dock.
   [FM DECISION] Robot A: previous task was not cancelled and robot is not at a home dock.
      Charge Task Creation calls: 1 (Expected > 0)

Native Database Simulation Completed.

