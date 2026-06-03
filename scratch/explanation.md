You caught the exact source of the illusion.

`C12` is **never actually removed from the database order records (`order_recs`)** while the robot is traveling between C12 and C10. 

The reason it looks like it was removed in some logs (like `horizon: ['C10', 'C9']`) is purely because of **in-memory slicing** inside Python, not because the database changed.

Here is the exact chain of events showing why this happens:

### 1. The Database DOES NOT Change
When the robot physically arrives at `C12`, the Fleet Manager **does not** issue a new order. It only issues a new order when it needs to grant the next block of nodes (e.g., when it reaches C10 and needs C9). 
Because no new order is published, **the database row remains exactly `['C12', 'C10', 'C9']`**.

### 2. Why `horizon` shows `['C10', 'C9']`
When `FmTrafficHandler` builds the state context for the robot, it reads the robot's physical location from the MQTT `state` message (`last_reached_node = 'C12'`). 
It then does this exact Python slice in memory:
```python
# checkps comes from the DB (['C12', 'C10', 'C9'])
# last_reached_node comes from the State Message ('C12')
horizon = checkps[checkps.index(last_reached_node) + 1:] 
# horizon = checkps[1:] -> ['C10', 'C9']
```
This is an **in-memory calculation**. It intentionally filters out nodes in the past so the traffic solver only looks ahead. It **does not** touch the database.

### 3. Why `Traffic Control` shows `['C12', 'C10']`
While `horizon` filters out the past, the `traffic_control_dict` loop **does not**. 
In `fetch_mex_data`, the code loops over the raw `order_recs` from the database and blindly adds **every single node where `released == True`**:
```python
nodes = order_rec[nodes_index+1] # This is STILL ['C12', 'C10', 'C9']
for point in waypoints:
    if release_state: # C12 is True, C10 is True, C9 is False
        traffic_control_dict[serial_number].append(node_id)
```
Because the database still has `C12` as released, it gets added to Traffic Control. **This is why C12 acts as a "ghost node" reserving space the robot has already left.**

### The "Magical" Removal
`C12` will finally disappear from `order_recs` (and thus from Traffic Control) **only when the robot reaches C10**. 
Once it reaches C10, `n_r` drops to 0, `Can_progress` becomes True, and the Fleet Manager calls `create_order`. 
`create_order` publishes a brand new order starting from `C10` (granting `C9`). This **overwrites the database row** with `['C10', 'C9']`. 
Only then does `order_recs` physically lose `C12`.

**Conclusion:**
There is no "magic" deletion. The `FmRobotSimulator` updates its own internal list, but the Fleet Manager ignores that. The Fleet Manager relies on the database, which stays statically at `['C12', 'C10', 'C9']` until the next formal order dispatch. We need to fix the `fetch_mex_data` loop to stop inserting past nodes into `Traffic Control`.
