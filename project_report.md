# OpenFMS Fleet Manager — Senior Engineering Audit Report

**Date:** 2026-02-25  
**Scope:** Scalability, Performance Bottlenecks, Redundancy, Analytics Correctness  
**Target:** Production readiness for 1,000-robot real-time factory deployment

---

## Table of Contents

1. [System Architecture Summary](#1-system-architecture-summary)
2. [Critical Bottlenecks (Ranked by Severity)](#2-critical-bottlenecks)
3. [Scalability Assessment: 1,000-Robot Target](#3-scalability-assessment)
4. [Redundancy Analysis](#4-redundancy-analysis)
5. [Analytics Deep-Dive: Correctness and Industry Alignment](#5-analytics-correctness)
6. [Risk Register](#6-risk-register)
7. [Recommended Remediation Roadmap](#7-remediation-roadmap)

---

## 1. System Architecture Summary

OpenFMS is a Python-based, MQTT-driven fleet management system implementing the VDA 5050 protocol. Its control flow is:

```
FmInterface (external dispatcher)
    └─► FmMain (MQTT hub + main loop thread)
            └─► FmScheduleHandler.manage_robot()  [called per-robot per cycle]
                    ├─► verify_robot_fitness()     [DB: connection / state tables]
                    ├─► order_handler.fetch_data() [DB: latest order per robot]
                    ├─► FmTrafficHandler.manage_traffic()
                    │       ├─► fetch_mex_data()   [in-mem state + DB orders]
                    │       └─► _handle_robot_traffic_status()  [conflict detection]
                    └─► visualization update + graph redraw
```

**Transport layer:** Mosquitto MQTT broker (single node).  
**Persistence:** PostgreSQL (single node, single connection per service).  
**State management:** In-memory `robot_state_cache` dict (recently improved from per-cycle DB reads).  
**Conflict detection:** Node-reservation-based (horizon/base locking), mutex groups for shared corridors.

---

## 2. Critical Bottlenecks

### 🔴 B1 — Sequential Main Loop with Blocking Sleep (CRITICAL)

**Location:** `FmMain.main_loop()` (FmMain.py:150–172)

```python
while True:
    i += 1
    if i % self.timer_interval == 0 ...:
        with self.lock:
            time.sleep(2.0)                          # ← BLOCKING SLEEP INSIDE LOCK
            for r_id in self.serial_numbers:         # ← SEQUENTIAL, NOT PARALLEL
                self.schedule_handler.manage_robot(...)
```

**Problem:** The entire management cycle is single-threaded and sequential. Each robot is processed one after the other. With N robots and a 2-second sleep inside the lock, a complete management cycle takes:

```
T_cycle = 2s + N × T_manage_robot
```

Where `T_manage_robot` includes at minimum one DB query (`fetch_all_data`), conflict detection, and graph redraw. At 2 robots this is acceptable. At 100 robots with ~200ms per robot:

```
T_cycle(100) ≈ 2s + 100 × 0.2s = 22 seconds per full cycle
T_cycle(1000) ≈ 2s + 1000 × 0.2s = 202 seconds per full cycle (3+ minutes!)
```

A robot at 1 m/s traverses 600+ meters between management decisions. This is operationally unusable for a real factory.

**Root cause:** The 2-second sleep was originally a debounce mechanism but blocks the entire fleet's management. The sequential loop was fine for prototype scale but does not parallelize.

**Fix:** Remove the sleep from inside the loop. Run per-robot management in a thread pool:

```python
from concurrent.futures import ThreadPoolExecutor

MAX_WORKERS = min(32, os.cpu_count() * 4)

def main_loop(self):
    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        while True:
            if self.serial_numbers and self.fleetname:
                futs = [
                    executor.submit(
                        self.schedule_handler.manage_robot,
                        self.fleetname, r_id, self.manufacturer, self.version
                    )
                    for r_id in list(self.serial_numbers)
                ]
                for f in futs:
                    f.result()     # propagate exceptions
            time.sleep(0.5)        # one global cycle throttle, outside the robot loop
```

**Note:** `manage_robot` must be made thread-safe. The shared `temp_fb_*` instance variables on `FmTrafficHandler` are the main re-entrancy hazard (see B3).

---

### 🔴 B2 — Per-Cycle DB Query for Orders (HIGH)

**Location:** `FmTrafficHandler.fetch_mex_data()` (FmTrafficHandler.py:1092)

```python
order_recs = self.task_handler.order_handler.fetch_all_data(f_id, m_id)
```

This query runs **every management cycle for every robot**:

```sql
SELECT DISTINCT ON (serial_number) *
FROM table_order
WHERE zone_set_id = %s AND manufacturer = %s
ORDER BY serial_number, timestamp DESC;
```

It fetches the latest order for every robot in the fleet on every call. With N robots and a management cycle every ~500ms:

- **2 robots:** 2 × 2 queries/s = 4 queries/s — fine
- **100 robots:** 100 × 2 queries/s = 200 queries/s — stressful for a single Postgres instance
- **1000 robots:** 1000 × 2 queries/s = 2,000 queries/s — Postgres collapses (practical limit ~500 simple queries/s on a single instance without connection pooling)

**Root cause:** Orders are only written by the FM itself. There is no reason to re-read them from DB on every cycle. They should be cached in memory and only invalidated when the FM writes a new order.

**Fix:** Maintain an in-memory `order_cache` dict keyed by `r_id` in `OrderPublisher`. Invalidate/update the cache entry immediately after every `INSERT`, `UPDATE`, or `DELETE` to the order table. `fetch_mex_data` reads from cache; DB is only consulted on cold-start or explicit cache miss.

```python
# In OrderPublisher.__init__:
self.order_cache = {}   # { r_id: latest_order_row }

# After every DB write:
self.order_cache[r_id] = new_row

# In fetch_all_data (or a new fetch_cached):
if r_id in self.order_cache:
    return self.order_cache[r_id]
```

---

### 🔴 B3 — Shared Mutable State on FmTrafficHandler (CRITICAL for Parallelism)

**Location:** `FmTrafficHandler._reset_temp_feedback()`, `_fetch_current_robot_data()`, `_handle_robot_traffic_status()`

`FmTrafficHandler` uses ~25 instance-level `temp_fb_*` variables as a scratchpad during each robot's management cycle:

```python
self.temp_fb_agv_position = ...
self.temp_fb_base = ...
self.temp_fb_horizon = ...
self.temp_fb_checkpoints = ...
# etc.
```

These are set at the start of each robot's cycle and read throughout. If two robots are processed concurrently (as required by the fix to B1), **Robot B's data will overwrite Robot A's data mid-processing**, causing incorrect traffic decisions, missed conflicts, or false collisions.

**Fix:** Convert the `temp_fb_*` variables to a local `RobotContext` dataclass passed as a parameter through the call chain. All functions that currently read `self.temp_fb_*` become pure functions taking a `ctx` argument. This enables full parallelism:

```python
@dataclass
class RobotContext:
    agv_position: list
    base: str
    horizon: list
    horizon_release: list
    checkpoints: list
    # ... etc.

def manage_traffic(self, f_id, r_id, m_id, v_id):
    ctx = self._fetch_current_robot_data(f_id, r_id, m_id)  # returns ctx
    self._handle_robot_traffic_status(f_id, r_id, m_id, v_id, ctx)
```

---

### 🟡 B4 — Conflict Detection O(N²) Traffic Control Scan (MEDIUM)

**Location:** `FmTrafficHandler.fetch_mex_data()` — traffic control list construction (FmTrafficHandler.py:1250–1350 approx.)

The `traffic_control` list is rebuilt from scratch every cycle by scanning all robots' current base nodes. Then for each robot, the entire `traffic_control` list is scanned to check for node occupancy (`next_stop_id in traffic_control`). The mutex group expansion also does a full group×traffic_control scan.

At N robots with M mutex groups of average size G:
- List membership check: O(N) per robot → O(N²) total
- Mutex expansion: O(G × N × M)

**Fix:** Replace the `traffic_control` list with a `set` for O(1) membership checks. The set is already semantically correct (no robot occupies two base nodes simultaneously):

```python
traffic_control = set()    # was: list
# ...
if next_stop_id in traffic_control:   # O(1) instead of O(N)
```

---

### 🟡 B5 — fetch_active_and_unassigned_tasks Full Table Scan with JSON Parsing (MEDIUM)

**Location:** `OrderPublisher.fetch_active_and_unassigned_tasks()` (order.py:952–1016)

```python
query = f"""
    SELECT order_id, nodes
    FROM {self.table_order}
    WHERE zone_set_id = %s AND manufacturer = %s
"""
```

This fetches **every order** (including completed and cancelled ones) from the DB and then filters in Python by parsing JSON `nodes` to find the task type. At scale this means:
- Fetching potentially millions of historical order rows
- Deserializing JSON for each row in Python
- Building sets from scratch every analytics call

**Fix 1:** Add a `status` column to the order table (`active`, `completed`, `cancelled`, `unassigned`). Query with `WHERE status = 'active'`.

**Fix 2 (interim):** Use `order_id LIKE '%_completed'` and `order_id LIKE '%_cancelled'` exclusions directly in SQL (already partially done in `fetch_completed_tasks`). Apply the same pattern here.

---

### 🟡 B6 — `busy_wait(0.05)` Busy Loop in Traffic Handler (LOW-MEDIUM)

**Location:** `FmTrafficHandler._handle_robot_traffic_status()` (FmTrafficHandler.py:636)

```python
self.busy_wait(0.05)

def busy_wait(self, duration_in_seconds):
    end_time = time.time() + duration_in_seconds
    while time.time() < end_time:
        pass                         # ← Spin loop, consumes 100% of one CPU core
```

A 50ms spin-lock on every robot management call. With 1000 robots processed sequentially, this is 50 seconds of pure CPU spin per cycle. Even with parallelism, 1000 threads each spinning for 50ms at the same time saturates CPU cores.

**Fix:** Replace with `time.sleep(0.05)`. The original concern was likely timer precision, but for this use case thread-sleep is perfectly adequate.

---

### 🟡 B7 — Graph Visualization Redrawn Every Management Cycle (MEDIUM)

**Location:** `FmScheduleHandler.manage_robot()` (FmScheduleHandler.py:~228)

```python
self.traffic_handler.task_handler.visualization_handler.terminal_graph_visualization()
```

This is called after **every single robot's management cycle** — building and rendering the full grid graph for every robot every cycle. At N robots per cycle this runs N times when it only needs to run once per cycle.

**Fix:** Move the visualization call outside the per-robot loop in `manage_robot`. Call it once after all robots have been processed in `FmMain.main_loop`.

---

## 3. Scalability Assessment

### Can OpenFMS control 1,000 robots in real-time today?

**No. Not even close.** Current architecture is fundamentally single-threaded with a 2-second blocking sleep in the main loop.

| Metric | 2 robots (current) | 100 robots | 1000 robots (target) |
|---|---|---|---|
| Cycle time | ~4s | ~22s | ~202s |
| DB queries/s | ~4 | ~200 | ~2,000 |
| Decision latency | 4s | 22s | >3 minutes |
| Busy-wait CPU | negligible | ~6s/cycle | ~50s/cycle |
| Practical usable? | ✅ Yes | ⚠️ Marginal | ❌ No |

### What's needed for 1,000 robots

| Requirement | Current State | Required Change |
|---|---|---|
| Parallel robot processing | Sequential, 1 thread | Thread pool (32–64 workers) |
| State management | In-memory cache ✅ (recent improvement) | Already good |
| Order management | DB query per cycle ❌ | In-memory order cache |
| Conflict detection | O(N) list scan | O(1) set operations |
| MQTT throughput | Single broker, 1 connection | MQTT cluster or multiple brokers per zone |
| DB layer | Single Postgres instance | Connection pool (PgBouncer), read replicas for analytics |
| Traffic handler thread safety | Not thread-safe (shared temp_fb_*) | RobotContext refactor |
| Visualization | Every robot, every cycle | Once per cycle |
| GIL limitation | Python GIL limits true parallelism | Use multiprocessing or asyncio for CPU-bound work |

### Realistic scalability ceiling per architecture tier

| Architecture | Robot Ceiling | Latency |
|---|---|---|
| Current (sequential, single thread) | ~5 robots | 2–10s |
| After B1+B2+B3 fixes (thread pool + cache) | ~100–200 robots | 0.5–2s |
| After all fixes + asyncio + PgBouncer | ~500 robots | 200–500ms |
| Distributed (zone-partitioned FMs + message bus) | 1000+ robots | <200ms |

For 1,000 robots, a **zone-partitioned deployment** is the correct architecture: multiple FM instances each managing a physical zone (e.g., floor, aisle), coordinating via a shared message bus (MQTT or Redis Streams) for cross-zone handoffs.

---

## 4. Redundancy Analysis

### R1 — `verify_robot_fitness` Called Every Cycle (Every Cycle DB Read)

**Location:** `FmScheduleHandler.manage_robot()` calls `verify_robot_fitness()` which calls `find_nearest_node()`.

`find_nearest_node` iterates the **entire itinerary** to find the closest node for every robot on every cycle. At N robots × K itinerary nodes this is O(N×K) per cycle. The robot's current node (`last_node_id`) is already known from the state cache — `find_nearest_node` is only needed when the robot's reported `lastNodeId` is empty. This is a cold-start edge case, not the common case.

**Fix:** In `verify_robot_fitness`, use `last_node_id` from the state cache directly when it's non-empty. Only call `find_nearest_node` as a fallback.

### R2 — `fm_analytics` Makes 6 Separate DB Queries Serially

In `fm_analytics()`:
1. `fetch_completed_tasks(cleared=True)` → DB query
2. `fetch_completed_tasks(cleared=False)` → DB query
3. `fetch_completed_tasks(cleared=True, task_type='charge')` → DB query
4. `fetch_active_and_unassigned_tasks()` → full table scan + JSON parse
5. `calculate_completed_delays()` → in-memory ✅
6. `compute_average_execution_duration()` → in-memory ✅
7. `compute_overall_throughput()` → in-memory ✅ but loops 120 times
8. `compute_robot_avg_latency()` → in-memory ✅
9. `compute_system_avg_latency()` → in-memory ✅
10. `compute_overall_idle_metrics()` → in-memory ✅

Queries 1–4 could be merged into a single SQL query using `CASE` expressions or CTEs. This reduces 4 DB round-trips to 1.

### R3 — `cancel_task` Still Uses `random.choice` for Home Dock

**Location:** `FmScheduleHandler.cancel_task()` (FmScheduleHandler.py:571)

```python
random_dock_id = random.choice(self.home_dock_loc_ids)
```

The same random dock selection bug that was fixed in `handle_transport_or_loop_task` still exists in `cancel_task`. When a task is cancelled, the robot is sent to a random home dock instead of its own.

**Fix:** Apply the same `loc_node_owner in home_dock_loc_ids` preference logic or track each robot's resident home dock.

### R4 — `compute_overall_throughput` Loops Over 120 Minutes Unconditionally

**Location:** `OrderPublisher.compute_overall_throughput()` (order.py:104–146)

```python
for minute in range(duration_minutes):     # duration_minutes = 120
    start_time = current_time - (duration_minutes - minute) * 60
    end_time = start_time + 60
    throughput = calculate_throughput(start_time, end_time)
```

`calculate_throughput` itself iterates over **all tasks for all robots** for each minute bucket:

```python
total_tasks = sum(
    1 for tasks in self.analytics_data.values()
    for task in tasks
    if start_time <= task["completion_timestamp"] < end_time
)
```

This is O(120 × total_tasks). With 1,000 robots completing 10 tasks each over 2 hours, this is O(120 × 10,000) = 1.2M iterations every analytics call.

**Fix:** Use a pre-sorted or bucketed structure. At analytics call time, sort completions once by timestamp, then use `bisect` to count completions per bucket in O(log N) rather than O(N) per bucket.

### R5 — `find_nearest_node` Iterates Full Itinerary Including Waitpoints

`find_nearest_node` skips `waitpoint` nodes but still iterates them. For large maps (hundreds of nodes), this is O(K) on every call. Maintaining a pre-built index of `{loc_id: coordinate}` for the non-waitpoint nodes would reduce this to O(1) lookup when `lastNodeId` is known.

---

## 5. Analytics Correctness

### A1 — `compute_average_execution_duration` ✅ Correct

**What it measures:** Mean task execution time per robot, computed as simple arithmetic mean over `analytics_data[robot_id]` entries.

**Formula:** `avg = sum(durations) / count`

**Correctness:** Mathematically sound. `analytics_data` is populated in `record_wait_event` when `completed=True`, using wall clock timestamps from task dispatch to completion.

**Industry alignment:** This is the standard KPI "Mean Cycle Time" or "Average Order Completion Time" in industrial WMS/WCS systems. ✅ Correct interpretation.

**Gap:** No confidence interval or percentile (p95, p99) is computed. For production use, a single mean is misleading when there are outliers (e.g., one very slow task skews the average significantly). Add `np.percentile(durations, [50, 95, 99])` to the output.

### A2 — `calculate_completed_delays` ⚠️ Misleading

**What it reports:** "Cumulative delay" = `cumulative_wait` field from the most recent completed order per robot within a 2-hour window.

**Problem 1 — Counts active robots, not total robots.** `num_robots` is the count of robots that have at least one completed order in the window. If a robot had no completed orders (initializing, idle, or on a very long task), it is not counted. This means "2 robots" does not mean the fleet has 2 robots — it means 2 robots completed at least one task.

**Problem 2 — `cumulative_wait` meaning is opaque.** The variable accumulates total wait time across all orders for a robot, but the function returns only the value from the *latest* completed order's `cumulative_wait`. This is actually a running total stored per order, not a per-order value. The naming is confusing and the log output `"cummulative delays: X"` doesn't clarify the unit.

**Industry alignment:** The correct metric is **Robot Utilization Rate** = (time spent executing tasks) / (total time). Or **Fleet Waiting Time** = sum of all wait events across all robots. The current calculation is neither clearly one nor the other.

**Fix:** Report separately:
- Per-robot total wait time (sum of all wait events)
- Per-robot utilization % = (active task time) / (session duration)
- Fleet-wide mean utilization

### A3 — `compute_overall_throughput` ⚠️ Near-Zero for Short Simulations

**What it measures:** Tasks completed per minute across 120 one-minute buckets.

**Problem:** For a 3–5 minute test scenario with 2 robots completing 2 tasks, 118 of the 120 buckets will show throughput = 0. The plot is essentially empty for the relevant simulation duration. This is not a correctness error, but the output is useless at simulation scale.

**Industry alignment:** Standard WMS throughput KPI is tasks/hour for the *active period only*. The function should compute throughput only over the actual simulation duration, not a fixed 120-minute window.

**Fix:** Replace the hardcoded `duration_minutes=120` with `actual_duration = (last_completion - first_dispatch) / 60` and use that as the window.

### A4 — `compute_robot_avg_latency` ✅ Correct (MQTT latency)

**What it measures:** Average MQTT state message latency per robot, computed from bucketed rolling windows.

**Correctness:** Mathematically correct. Uses incremental sum/count per time bucket to compute rolling average.

**Industry alignment:** This maps to "Communication Latency" or "State Update Frequency" — a valid real-time monitoring KPI. ✅

**Gap:** Latency is computed from `state_timestamp` arrival vs. message timestamp, but MQTT timestamps depend on robot clock sync. Without NTP synchronization between robots and the server, this metric may be inaccurate by ±seconds.

### A5 — `compute_overall_idle_metrics` ✅ Correct but Limited

**What it measures:** Average idle time per robot (time spent at home dock with no active order).

**Correctness:** Logic is sound — idle time accumulates when `last_node_id in home_dock_loc_ids`.

**Industry alignment:** "Fleet Idle Rate" is a standard KPI. ✅

**Gap:** No distinction between "intentionally idle" (task complete, waiting for next dispatch) and "stuck/error idle" (robot failed to complete a task and returned home). Both look identical in the current metric.

### A6 — `collision_tracker` ✅ Correct in Scope

**What it counts:** Number of times `_handle_active_mex_conflict` was entered — i.e., traffic conflicts detected.

**Correctness:** Counts conflict *detection* events, not actual physical collisions (which don't happen if the FM is working correctly). The label "detected target collisions" is accurate to its implementation.

**Recommendation:** Rename to `conflict_detections` in the output to avoid implying physical collision. Also add: `conflict_resolution_rate` = fraction of conflicts successfully resolved without robot halt.

---

## 6. Risk Register

| ID | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | GIL prevents true CPU parallelism in thread pool | High | High | Migrate hot path to asyncio or multiprocessing |
| R2 | Single MQTT broker is SPOF | High | Critical | Deploy MQTT cluster (EMQX, VerneMQ) with load balancing |
| R3 | Single Postgres instance falls over at 1000 robots | High | Critical | PgBouncer connection pooling + read replica for analytics |
| R4 | temp_fb_* race condition causes wrong orders sent | High (if parallelized) | Critical | RobotContext refactor before enabling parallelism |
| R5 | cancel_task sends robot to wrong home dock | Medium | Medium | Apply same preference fix as transport task |
| R6 | Clock skew makes latency metrics inaccurate | Medium | Low | Enforce NTP sync on all robot simulators/real robots |
| R7 | Analytics throughput metric is misleading | High | Low | Fix window duration to match actual simulation time |

---

## 7. Remediation Roadmap

### Phase 1 — Correctness (Do Now)

| Issue | File | Fix |
|---|---|---|
| `cancel_task` random dock | `FmScheduleHandler.py:571` | Apply same `loc_node_owner` preference |
| `busy_wait` spin loop | `FmTrafficHandler.py:636` | Replace with `time.sleep()` |
| Visualization per-robot | `FmScheduleHandler.py:228` | Move outside per-robot loop |
| Analytics throughput window | `order.py:104` | Use actual session duration |
| Analytics delay metric label | `FmScheduleHandler.py:698` | Rename and add units |

### Phase 2 — Performance (Next Sprint)

| Issue | File | Fix |
|---|---|---|
| Order cache | `order.py` + `FmTrafficHandler.py` | In-memory `order_cache` dict |
| Traffic control `set` | `FmTrafficHandler.py` | `list` → `set` for O(1) lookup |
| Throughput O(120×N) | `order.py:107` | Pre-sort + bisect bucketing |
| Merge analytics DB queries | `FmScheduleHandler.py:678–721` | Single CTE query for all counts |

### Phase 3 — Architecture (Before Production at Scale)

| Issue | Change |
|---|---|
| Sequential main loop | Thread pool with `RobotContext` refactor |
| `temp_fb_*` shared state | `RobotContext` dataclass — passed through call chain |
| Single MQTT broker | EMQX cluster, zone-partitioned topic namespaces |
| Single Postgres | PgBouncer + read replica for analytics |
| Python GIL | Asyncio event loop for I/O-bound path; or zone-partitioned subprocesses |
| 1000+ robots | Zone-partitioned FM instances (e.g., 10 FM instances × 100 robots each) |

### Phase 4 — Observability (Before Production)

- Add structured metrics endpoint (Prometheus `/metrics`)
- Export per-robot: utilization %, mean cycle time, p95 cycle time, wait time, latency
- Export fleet-wide: throughput (tasks/hr), conflict rate, idle rate
- Alert on: robot offline >30s, conflict not resolved >5 cycles, DB query latency >100ms

---

## Completed Tasks (as of this report)

- [x] Fix `from_loc='A12'` default in `fm_dispatch_task` (FmMain.py)
- [x] Fix inverted `blockingType` logic in `FmRobotSimulator` (downloadMap now correctly updates `active_map`)
- [x] Guard elevator check for `None` active_map (FmTrafficHandler.py ×2)
- [x] Fix `pull_policy: if_not_present` for offline Docker operation
- [x] Fix stale node highlights in visualization (vis_release flags + active_horizons.pop)
- [x] Add periodic `fm_analytics` call in FmInterface main loop
- [x] Add startup pre-flight cleanup in `run_openfms.sh` (stale containers + log files)
- [x] Fix robot color stability (deterministic hash-based color assignment)
- [x] Fix home dock preference in transport task (prefer `loc_node_owner`)

## In Progress / Remaining

- [x] `cancel_task` still uses `random.choice` for dock selection
- [x] `busy_wait` spin loop not yet replaced with `time.sleep`
- [x] Visualization redrawn per-robot per-cycle (not per-cycle)
- [ ] Thread pool parallelism (requires `RobotContext` refactor first)
- [ ] Order cache (in-memory to eliminate per-cycle DB query)
- [ ] Traffic control `list` → `set`
- [x] Analytics metrics calibration (percentiles, utilization %, actual-duration throughput)
# Technical Implementation Plan

## Goal Description
The objective is to resolve a critical bug where the system "waits forever as it claims it never docked." The investigation revealed two separate synchronization bugs inside `FmRobotSimulator.py` that prevent the Fleet Manager (`FmTrafficHandler`) from accurately ascertaining message states.

**Root Causes:**
1. **Skipped Node Action Processing**: When the robot arrives at a node (e.g., `C17`), `FmRobotSimulator` was only iterating over and processing the `actions` array (which contains the required `dock` action) IF that node was the *absolute last* node in its current order. If the Fleet Manager provided a lookahead horizon (e.g., stopping at `C17` but `C18` is in the array as `released: False`), the robot would arrive at `C17`, pause, and skip the action processing entirely. This causes `dock_action_done` to never become `True` inside the Fleet Manager.
2. **Missing Terminal State Publish**: In `handle_instant_action` (processing `pick` or `drop`), the loop correctly sleeps and transitions the action state to `FINISHED`. However, it breaks out of the loop *without* calling `self.publish_state()` immediately. The robot resumed motion immediately, relying on a generic background timer thread to eventually send the `FINISHED` state, which causes race conditions if the Fleet Manager sends a new order concurrently.

## Proposed Changes

### OpenFMS Core Implementation
---

#### [MODIFY] FmRobotSimulator.py
- **Node Action Execution**: Relocate the logic block that parses `self.target_node["actions"]` to execute *before* the completed node is popped from the order array. This ensures that any node possessing actions (such as a checkpoint with a `dock` action) evaluates those actions the moment it arrives, regardless of whether there are more nodes ahead in the order.
- **Instant Action State Guarantee**: In `handle_instant_action`, inject a definitive `self.publish_state()` call right after the loop breaks to guarantee the `FINISHED` status is dispatched synchronously before setting `self.instant_action = False` (which allows the robot to drive again).

## Verification Plan

### Manual Verification
- Run the simulation environment.
- Observe high-priority or waitpoint scenarios where docking sequences (pick/drop) occur. The Fleet Manager should now immediately register the `dock` completion, trigger the instant action, and receive the synchronous `FINISHED` state, advancing the sequence reliably without stalling forever.
