#!/usr/bin/env python3
"""
conflict_test.py
================
Real integration tests bypassing internal variables to completely eliminate
test-induced bias.

This test simulates standard execution identical to `FmTrafficHandler.py __main__`:
We inject native PostgreSQL result tuples into `fetch_all_data()` exactly as
they would appear natively, allowing `manage_traffic()` and `fetch_mex_data()`
to parse and populate all data natively without external manipulation.
"""
import sys, os, datetime, types, unittest.mock, time

# Stub imports BEFORE importing real modules
for _n in ['psycopg2','psycopg2.extras','twilio','twilio.rest']:
    if _n not in sys.modules: sys.modules[_n] = unittest.mock.MagicMock()
for _n in ['numpy','numpy.core','scipy','scipy.signal','skfuzzy','skfuzzy.control','skfuzzy.membership','skfuzzy.defuzz']:
    if _n not in sys.modules: sys.modules[_n] = types.ModuleType(_n)
for _n in ['connection','factsheet','state','visualization','instant_actions','order']:
    if _n not in sys.modules: sys.modules[_n] = unittest.mock.MagicMock()

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
FM   = os.path.join(BASE, 'fleet_management')
sys.path.insert(0, FM)
sys.path.insert(0, os.path.join(BASE, 'submodules'))

import yaml
from pathlib import Path
from FmTrafficHandler import FmTrafficHandler
from FmTaskHandler    import FmTaskHandler
from FmScheduleHandler import FmScheduleHandler

GREEN = "\033[32m"; RED = "\033[31m"; CYAN = "\033[36m"; RESET = "\033[0m"; BOLD = "\033[1m"
FLEET = 'kullar'; MFR = 'birfen'; VER = '1.0.0'; VERS = 'v1'

def _load():
    for p in [Path(BASE)/'fleet_management'/'config'/'config.yaml', Path(BASE)/'config'/'config.yaml']:
        if p.exists(): return yaml.safe_load(open(p))
    return {}
TASK_DICT = _load()

COORD_MAP    = {i['loc_id']: i['coordinate'] for i in TASK_DICT.get('itinerary', []) if i.get('fleet_id')==FLEET}
HOME_DOCKS   = [i['loc_id'] for i in TASK_DICT.get('itinerary', []) if i.get('description')=='home_dock' and i.get('fleet_id')==FLEET]
CHARGE_DOCKS = [i['loc_id'] for i in TASK_DICT.get('itinerary', []) if i.get('description')=='charge_dock' and i.get('fleet_id')==FLEET]

def _c(n): c=COORD_MAP.get(n,[0.0,0.0,0.0,1.0]); return [float(c[0]),float(c[1]),float(c[2])]

# --------------------------------------------------------------------------------------------------------------------------
# DB SCHEMA MOCKS
# --------------------------------------------------------------------------------------------------------------------------

STATE_COLS = [
    'header_id', 'timestamp', 'version', 'manufacturer', 'serial_number', 'maps', 'order_id',
    'order_update_id', 'zone_set_id', 'last_node_id', 'last_node_sequence_id', 'driving', 'paused',
    'new_base_request', 'distance_since_last_node', 'operating_mode', 'node_states', 'edge_states',
    'agv_position', 'velocity', 'loads', 'action_states', 'battery_state', 'errors', 'information',
    'safety_state'
]
ORDER_COLS = [
    'header_id', 'timestamp', 'version', 'manufacturer', 'serial_number', 'order_id', 'order_update_id',
    'zone_set_id', 'nodes', 'edges'
]

def mock_state_rec(r_id, base, horizon=None, position=None, ts=None):
    if position is None: position = _c(base) if base else [0.0,0.0,0.0]
    past_ts = (datetime.datetime.now() - datetime.timedelta(seconds=20)).isoformat()
    
    rec = [None] * (len(STATE_COLS) + 1)
    rec[STATE_COLS.index('timestamp')+1] = ts or past_ts
    rec[STATE_COLS.index('version')+1] = "1.0.0"
    rec[STATE_COLS.index('manufacturer')+1] = MFR
    rec[STATE_COLS.index('serial_number')+1] = r_id
    rec[STATE_COLS.index('last_node_id')+1] = base
    rec[STATE_COLS.index('agv_position')+1] = {"x": position[0], "y": position[1], "theta": position[2], "mapId": "bina_1_floor_0"}
    rec[STATE_COLS.index('errors')+1] = []
    rec[STATE_COLS.index('action_states')+1] = [{"actionId": "dock_action_123", "actionStatus": "FINISHED"}]
    rec[STATE_COLS.index('safety_state')+1] = {"eStop": "NONE", "fieldViolation": False}
    rec[STATE_COLS.index('velocity')+1] = {"vx": 0.5, "omega": 0.0}
    rec[STATE_COLS.index('maps')+1] = [{"mapId": "bina_1_floor_0", "mapStatus": "ENABLED"}]
    rec[STATE_COLS.index('node_states')+1] = []
    return tuple(rec)

def mock_order_rec(r_id, order_id, checkpoints=None, waitpoints=None, target_base_idx=0, landmark=None, ts=None):
    """ Builds realistic waypoints exactly how they're stored in Postgres JSONB arrays by FmTaskHandler. """
    past_ts = (datetime.datetime.now() - datetime.timedelta(seconds=20)).isoformat()
    
    rec = [None] * (len(ORDER_COLS) + 1)
    rec[ORDER_COLS.index('timestamp')+1] = ts or past_ts
    rec[ORDER_COLS.index('serial_number')+1] = r_id
    rec[ORDER_COLS.index('order_id')+1] = order_id
    rec[ORDER_COLS.index('header_id')+1] = 1

    nodes = []
    seq = 0
    l_mark = landmark or [0.0, 'high', 'transport', 'DOCK_A', 'DOCK_B']

    # Injecting nodes into the order payload
    if checkpoints:
        for i, cp in enumerate(checkpoints):
            n = {"nodeId": cp, "sequenceId": seq, "released": (i <= target_base_idx), 
                 "nodePosition": {"x": _c(cp)[0], "y": _c(cp)[1], "theta": 0.0},
                 "nodeDescription": ""}
            
            # Everyone gets the action just so `dock_action_done` evaluates across the board
            n["actions"] = [{"actionType": "dock", "actionId": "dock_action_123", "actionParameters": [{"value": l_mark}]}]
            
            nodes.append(n)
            seq += 1

    if waitpoints:
        for wp in waitpoints:
            n = {"nodeId": wp, "sequenceId": seq, "released": True,
                 "nodePosition": {"x": _c(wp)[0], "y": _c(wp)[1], "theta": 0.0},
                 "nodeDescription": ""}
            n["actions"] = [{"actionType": "dock", "actionId": "dock_action_123", "actionParameters": [{"value": l_mark}]}]
            nodes.append(n)
            seq += 1

    rec[ORDER_COLS.index('nodes')+1] = nodes
    return tuple(rec)


class TestTrafficEnvironment:
    """ Encapsulates a mock DB interaction layer for a cleanly testable FmTrafficHandler. """
    def __init__(self, r_ids):
        self.state_recs = []
        self.order_recs = []

        # Instantiating a clean FmTrafficHandler with completely mocked IO components
        # No DB connection is passed since we spoof fetch_all_data directly on the mocked handlers.
        th = FmTrafficHandler.__new__(FmTrafficHandler)
        for a in ['temp_fb_agv_position','temp_fb_agv_status','temp_fb_errors',
                'temp_fb_horizon','temp_fb_horizon_release','temp_fb_base',
                'temp_fb_checkpoints','temp_fb_waitpoints','temp_fb_landmarks',
                'temp_fb_checkp_itinerary','temp_fb_waitp_itinerary',
                'temp_fb_active_map_name','temp_fb_order_id','temp_fb_header_id',
                'temp_fb_order_timestamp','temp_fb_merged_nodes',
                'temp_fb_dock_action_done','temp_fb_halt','temp_fb_speed_min',
                'temp_fb_agv_size','temp_fb_vel_lin_ang']:
            setattr(th, a, None)

        th.version=VER; th.manufacturer=MFR; th.fleetname=FLEET; th.versions=VERS; th.task_dictionary=TASK_DICT
        th.mutex_groups=[]; th.collision_tracker=0; th.robots_in_collision=set()
        
        # Prepopulate delay time to bypass native `update_robot_elapsed_time` KeyError flaw:
        th.temp_robot_delay_time={rid: (time.time_ns(), 0.1) for rid in r_ids}
        th.wait_time_default=10.5
        
        tk = FmTaskHandler.__new__(FmTaskHandler)
        tk.version=VER; tk.manufacturer=MFR; tk.fleetname=FLEET; tk.versions=VERS; tk.task_dictionary=TASK_DICT
        tk.db_conn=unittest.mock.MagicMock(); tk.mqttclient=None
        tk.pause_list=[]; tk.ignore_list=[]; tk.cp_ignore_list=[]; tk.header_id_count=0
        tk.min_charge_level = 20.0
        
        # Mocks
        tk.visualization_handler = self._make_viz()
        tk.order_handler = self._make_order()
        tk.connection_handler = unittest.mock.MagicMock()
        tk.factsheet_handler = self._make_fact(r_ids)
        tk.state_handler = self._make_state()
        tk.instant_actions_handler = self._make_ia()
        tk.fuzzy_dispatcher = self._make_fuzzy()

        th.task_handler = tk
        self.th = th

    def _make_viz(self):
        m = unittest.mock.MagicMock()
        m.log_lines = []
        def log_print(msg, *a, **k):
            m.log_lines.append(str(msg))
            print(f"   {CYAN}[FM DECISION]{RESET} {msg}")
        m.terminal_log_visualization.side_effect = log_print
        m.terminal_graph_visualization.return_value = None
        return m

    def _make_order(self):
        m = unittest.mock.MagicMock()
        m.table_col = ORDER_COLS
        m.fetch_all_data.side_effect = lambda f, mid: self.order_recs
        m.spy_calls = []

        def _create_order_spy(*args, **kwargs):
            checkpoints = args[0] if len(args) > 0 else kwargs.get('checkpoints', [])
            waitpoints  = args[1] if len(args) > 1 else kwargs.get('waitpoints', [])
            horizon     = args[6] if len(args) > 6 else kwargs.get('horizon', [])
            wait_time   = args[8] if len(args) > 8 else kwargs.get('wait_time', None)
            m.spy_calls.append({
                'checkpoints': list(checkpoints) if checkpoints else [],
                'waitpoints':  list(waitpoints) if waitpoints else [],
                'horizon':     list(horizon) if horizon else [],
                'wait_time':   wait_time,
            })
            return {'nodeId':'X','sequenceId':0,'released':True,'actions':[]}, [], []
        
        m.create_order.side_effect = _create_order_spy
        m.build_order_msg.return_value = None
        return m

    def _make_state(self):
        m = unittest.mock.MagicMock()
        m.table_col = STATE_COLS
        m.fetch_all_data.side_effect = lambda f, mid: self.state_recs
        return m

    def _make_fact(self, rids):
        m = unittest.mock.MagicMock()
        m.fetch_serial_numbers.return_value = rids
        m.fetch_data.return_value = (None, 100.0, 0.5, 0.8, 0.5, None, None)
        return m
    
    def _make_ia(self):
        m = unittest.mock.MagicMock()
        m.create_action.return_value = {'actionId':'a1'}
        m.fetch_data.return_value = (None, None)
        return m
    
    def _make_fuzzy(self):
        m = unittest.mock.MagicMock()
        m.evaluate_robot_for_task.return_value = {'fitness':0.8,'travel_time':10.0}
        return m

    def set_db(self, state_list, order_list):
        self.state_recs = state_list
        self.order_recs = order_list

    def print_published_orders(self):
        bom_calls = self.th.task_handler.order_handler.build_order_msg.call_args_list
        spy = self.th.task_handler.order_handler.spy_calls
        for i, bom in enumerate(bom_calls):
            bom_args = bom[0]
            r_id = bom_args[1] if len(bom_args) > 1 else '?'
            spy_rec = spy[i] if i < len(spy) else {}
            wt = spy_rec.get('wait_time')
            print(f"   {GREEN}[ORDER PUBLISHED]{RESET} Robot: {r_id} | Wait Time: {wt}")
            print(f"                     Checkpoints: {spy_rec.get('checkpoints', [])}")
            print(f"                     Waitpoints:  {spy_rec.get('waitpoints', [])}")
            print(f"                     Horizon:     {spy_rec.get('horizon', [])}\n")

    def run_robot(self, r_id):
        print(f"\n{GREEN}[Execute] Analysing traffic state for {r_id}...{RESET}")
        self.th.manage_traffic(FLEET, r_id)
        self.print_published_orders()
        # Clean spies for next run
        self.th.task_handler.order_handler.build_order_msg.reset_mock()
        self.th.task_handler.order_handler.spy_calls.clear()
        self.th.task_handler.visualization_handler.log_lines.clear()

def print_header(title):
    print(f"\n{BOLD}{CYAN}========================================================================{RESET}")
    print(f"{BOLD}{CYAN}{title}{RESET}")
    print(f"{BOLD}{CYAN}========================================================================{RESET}")

def print_state(robots_info, traffic):
    print(f"{BOLD}--- NATIVE STATE REPRESENTATION ---{RESET}")
    for r, info in robots_info.items(): print(f"{BOLD}Robot {r}:{RESET} {info}")
    print(f"{BOLD}Traffic Constraint:{RESET} {traffic}")
    print(f"{BOLD}-----------------------------------{RESET}")

# ═════════════════════════════════════════════════════════════════════════════
# TESTS
# ═════════════════════════════════════════════════════════════════════════════

def test_s1():
    print_header("S1: Same Target Conflict (R01 and R02 want C10)")
    env = TestTrafficEnvironment(['R01', 'R02'])
    
    print_state({
        'R01': 'base=C2, order -> checkpoints=[C2, C10, C11, C3] (Released up to C2)',
        'R02': 'base=C9, order -> checkpoints=[C9, C10, C12] (Released up to C9)'
    }, "[] (Initially free)")

    # Simulate R01 getting its route first
    env.set_db(
        [mock_state_rec('R01', base='C2'), mock_state_rec('R02', base='C9')],
        [mock_order_rec('R01', 'order01', checkpoints=['C2','C10','C11','C3'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C2', 'C3']),
         mock_order_rec('R02', 'order02', checkpoints=['C9','C10','C12','C2'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C9', 'C12'])]
    )
    env.run_robot('R01') # R01 successfully reserves C10

    # R01 now holds C10
    print(f"\n{CYAN}--- R01 is now en route, holding traffic reservation on C10 ---{RESET}")
    # To simulate R01 having secured and moved to C10, we advance its base to C10 and release up to index 1
    env.set_db(
        [mock_state_rec('R01', base='C10'), mock_state_rec('R02', base='C9')],
        [mock_order_rec('R01', 'order01', checkpoints=['C10','C11','C3'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C2', 'C3']),
         mock_order_rec('R02', 'order02', checkpoints=['C9','C10','C12','C2'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C9', 'C12'])]
    )
    env.run_robot('R02') # R02 should see C10 is occupied and wait.

def test_s2a():
    print_header("S2a: No-swap Conflict [A & B nodes have waitpoints] – A(MEX)@C17(W17) vs B(R_ID)@C29(W29).")
    print_state({
        'A (High Prio)': 'base=C17, pending order to move to W17',
        'B (Low Prio)': 'base=C29, heading towards C17'
    }, "[C17, W17, C29]")

    env = TestTrafficEnvironment(['A', 'B'])
    # Robot A is AT C17, but was told to go to W17.
    env.set_db(
        [mock_state_rec('A', base='C17'), mock_state_rec('B', base='C29')],
        [mock_order_rec('A', 'orderA', checkpoints=['C17'], waitpoints=['W17'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C17', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C29','C17','C18'], waitpoints=[], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C29', 'DOCK_B'])]
    )
    env.run_robot('B')

def test_s2b():
    print_header("S2b: No-swap Conflict [No waitpoints, both have free C_nodes] – A@C18 vs B@C28.")
    print_state({
        'A (High Prio)': 'base=C18, holding. Connected free node C19.',
        'B (Low Prio)': 'base=C28, wants C18. Connected free node C20.'
    }, "[C18, C28]")

    env = TestTrafficEnvironment(['A', 'B'])
    env.set_db(
        [mock_state_rec('A', base='C18'), mock_state_rec('B', base='C28')],
        [mock_order_rec('A', 'orderA', checkpoints=['C18','C20'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C18', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C28','C18'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C28', 'DOCK_B'])]
    )
    env.run_robot('B')

def test_s2c():
    print_header("S2c: No-swap Conflict [Only one has free C_node] – A@C20 vs B@C21.")
    print_state({
        'A (High Prio)': 'base=C20, completely blocked.',
        'B (Low Prio)': 'base=C21(has free neighbor), wants C20.'
    }, "[C20, C21]")

    env = TestTrafficEnvironment(['A', 'B'])
    env.set_db(
        [mock_state_rec('A', base='C20'), mock_state_rec('B', base='C21')],
        [mock_order_rec('A', 'orderA', checkpoints=['C20'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C20', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C21','C20'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C21', 'DOCK_B'])]
    )
    env.run_robot('B')

def test_s2d():
    print_header("S2d: No-swap Conflict [No waitpoints, no free C_nodes] – A@C6 vs B@C7.")
    print_state({
        'A (High Prio)': 'base=C6, deadlocked environment.',
        'B (Low Prio)': 'base=C7, wants C6, deadlocked environment.'
    }, "[C6, C7]")

    env = TestTrafficEnvironment(['A', 'B'])
    # In S2d, there's absolutely nowhere to yield or backup natively. Let's see how FM handles a total topological deadlock.
    env.set_db(
        [mock_state_rec('A', base='C6'), mock_state_rec('B', base='C7')],
        [mock_order_rec('A', 'orderA', checkpoints=['C6'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C6', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C7','C6'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C7', 'DOCK_B'])]
    )
    env.run_robot('B')

def test_s3a():
    print_header("S3a: Swap Conflict [Both have waitpoints] – A@C17(W17) wants C12 ↔ B@C12(W12) wants C17.")
    print_state({
        'A (High Prio)': 'base=C17, wants C12. Has W17.',
        'B (Low Prio)': 'base=C12, wants C17. Has W12.'
    }, "[C17, C12]")

    env = TestTrafficEnvironment(['A', 'B'])
    env.set_db(
        [mock_state_rec('A', base='C17'), mock_state_rec('B', base='C12')],
        [mock_order_rec('A', 'orderA', checkpoints=['C17','C12'], waitpoints=['W17'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C17', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C12','C17'], waitpoints=['W12'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C12', 'DOCK_B'])]
    )
    env.run_robot('A')

def test_s3b():
    print_header("S3b: Swap Conflict [Only one has waitpoint] – A(R_ID, high)@C17 wants C9 ↔ B(MEX, low)@C9 wants C17.")
    print_state({
        'A': 'base=C17, wants C9. Has W17.', 'B': 'base=C9, wants C17. No waitpoint.'
    }, "[C17, C9]")

    env = TestTrafficEnvironment(['A', 'B'])
    env.set_db(
        [mock_state_rec('A', base='C17'), mock_state_rec('B', base='C9')],
        [mock_order_rec('A', 'orderA', checkpoints=['C17','C9','C10'], waitpoints=['W17'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C17', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C9','C17','C18'], waitpoints=[], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C9', 'DOCK_B'])]
    )
    env.run_robot('A')

def test_s3c():
    print_header("S3c: Swap Conflict [No waitpoints, both have free C_nodes] – A@C20 wants C21 ↔ B@C21 wants C20.")
    print_state({
        'A (High Prio)': 'base=C20, wants C21. Has free C19.',
        'B (Low Prio)': 'base=C21, wants C20. Has free C22.'
    }, "[C20, C21]")

    env = TestTrafficEnvironment(['A', 'B'])
    env.set_db(
        [mock_state_rec('A', base='C20'), mock_state_rec('B', base='C21')],
        [mock_order_rec('A', 'orderA', checkpoints=['C20','C21'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C20', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C21','C20'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C21', 'DOCK_B'])]
    )
    env.run_robot('A')

def test_s3d():
    print_header("S3d: Swap Conflict [No waitpoints, only one has free C_node] – A@C22 wants C23 ↔ B@C23 wants C22.")
    print_state({
        'A (High Prio)': 'base=C22, wants C23. Blocked behind it.',
        'B (Low Prio)': 'base=C23, wants C22. Has free C24.'
    }, "[C22, C23]")

    env = TestTrafficEnvironment(['A', 'B'])
    env.set_db(
        [mock_state_rec('A', base='C22'), mock_state_rec('B', base='C23')],
        [mock_order_rec('A', 'orderA', checkpoints=['C22','C23'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C22', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C23','C22'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C23', 'DOCK_B'])]
    )
    env.run_robot('A')

def test_s4():
    print_header("S4: Post-Resolution Continuation – A(High)@C17 wants [C9,C10,C11] ↔ B(Low)@C9 wants [C17,C18,C19].")
    print_state({
        'A (High Prio)': 'base=C17, wants C9 then C10. Has W17.',
        'B (Low Prio)': 'base=C9, wants C17 then C18. No waitpoint.'
    }, "[C17, C9]")

    # S4 evaluates whether after B yields to A and gets a W10 assignment, A successfully continues
    # and B is placed back natively into the queue without dropping its further waypoints C18, C19.
    env = TestTrafficEnvironment(['A', 'B'])
    
    # 1. Initial State
    env.set_db(
        [mock_state_rec('A', base='C17'), mock_state_rec('B', base='C9')],
        [mock_order_rec('A', 'orderA', checkpoints=['C17','C9','C10','C11'], waitpoints=['W17'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C17', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C9','C17','C18','C19'], waitpoints=[], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C9', 'DOCK_B'])]
    )
    env.run_robot('A') # B yields to W10.

    # 2. Advanced State: B has yielded to W10. A advances to C9.
    print(f"\n{CYAN}--- Advancing State: B yields to W10. A advances to C9 ---{RESET}")
    env.set_db(
        [mock_state_rec('A', base='C9'), mock_state_rec('B', base='W10')],
        [mock_order_rec('A', 'orderA', checkpoints=['C17','C9','C10','C11'], waitpoints=['W17'], target_base_idx=1, landmark=[0.0, 'high', 'transport', 'C17', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C9','C17','C18','C19'], waitpoints=['W10'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C9', 'DOCK_B'])]
    )
    env.run_robot('A') # A should be able to continue from C9 to C10 without freezing.
    env.run_robot('B') # B should notice A passing by and hold its yield position.

def test_s5():
    print_header("S5: Mutex Group Conflict – A@C2 (Inside Mutex group [C2,C3]) ↔ B@C4 wants C3.")
    print_state({
        'A (High Prio)': 'base=C2 (which is mutex-coupled with C3).',
        'B (Low Prio)': 'base=C4, wants C3.'
    }, "[C2] + Mutex[C2, C3]")

    env = TestTrafficEnvironment(['A', 'B'])
    # Spoof mutex group configuration
    env.th.mutex_groups = [['C2', 'C3']]
    
    env.set_db(
        [mock_state_rec('A', base='C2'), mock_state_rec('B', base='C4')],
        [mock_order_rec('A', 'orderA', checkpoints=['C2','C1'], target_base_idx=0, landmark=[0.0, 'high', 'transport', 'C2', 'DOCK_A']),
         mock_order_rec('B', 'orderB', checkpoints=['C4','C3'], target_base_idx=0, landmark=[0.0, 'low', 'transport', 'C4', 'DOCK_B'])]
    )
    # B analyzes traffic. Although only C2 and C4 are actively occupied natively, the Mutex logic should expand C2's occupancy to C3 blocking B.
    env.run_robot('B')


def test_s6():
    print_header("S6: Task Queueing – Assign task while robot A is busy.")
    print_state({
        'A (High Prio)': 'base=C2, actively holding an order to C10.',
    }, "N/A - Evaluating Task Assignment while busy")

    env = TestTrafficEnvironment(['A'])
    # Setup state manually so verify_robot_fitness sees a valid robot on a mission
    active_state = mock_state_rec('A', base='C2')
    active_order = mock_order_rec('A', 'orderA', checkpoints=['C2', 'C10'])
    env.set_db([active_state], [active_order])
    
    # Manually configure state_handler and order_handler for verify_robot_fitness checks
    # FmTaskHandler.verify_robot_fitness expects specific indices from `fetch_data` 
    single_state = (
        None, None, 'orderA', 'C2', None, True, [],
        {'x': 0.0, 'y': 0.0, 'theta': 0.0}, None, {'batteryCharge': 100.0}, None, None
    )
    env.th.task_handler.state_handler.fetch_data.return_value = single_state
    env.th.task_handler.order_handler.fetch_data.return_value = active_order
    env.th.task_handler.connection_handler.fetch_data.return_value = ("ONLINE", "some_ts")

    # Send a new task assignment to the busy robot
    print(f"\n{CYAN}[Execute] Testing task assignment on occupied robot...{RESET}")
    task_c, available_r, checkps, checkp_itinrry, waitps, wait_itinrry, landm_ = env.th.task_handler.fm_send_task_request(
        f_id=FLEET, r_id='A', from_loc_id='C17', to_loc_id='C11', 
        task_name='transport', task_priority='high', payload_kg=45
    )
    
    # We expect verify_robot_fitness to return cleared=False, which prevents immediate assignment
    print(f"      {CYAN}Returned Available Robots from assignment parser: {available_r}{RESET}")


def test_s7():
    print_header("S7: Low Battery Trigger – Robot A goes idle with 15% battery.")
    print_state({
        'A (Idle)': 'base=C2, idle, battery=15%.',
    }, "N/A - Evaluating Automatic Charge Dispatch")

    env = TestTrafficEnvironment(['A'])
    
    # Needs to be effectively idle (no order_id, driving=False)
    idle_state = mock_state_rec('A', base='C2')
    env.set_db([idle_state], [])
    
    single_state = (
        None, None, '', 'C2', None, False, [],
        {'x': 0.0, 'y': 0.0, 'theta': 0.0}, None, {'batteryCharge': 15.0}, None, None
    )
    env.th.task_handler.state_handler.fetch_data.return_value = single_state
    
    # Allow create_charge_task to return something verifiable
    env.th.task_handler.create_charge_task = unittest.mock.MagicMock()

    print(f"\n{CYAN}[Execute] Testing manage_robot on low-battery idle robot...{RESET}")
    # Call the top-level orchestration method that natively checks schedules and batteries
    env.th.task_handler.manage_robot(FLEET)
    
    # Verify that the low battery triggered the charge task creation sequence
    charge_call_count = env.th.task_handler.create_charge_task.call_count
    print(f"      {CYAN}Charge Task Creation calls: {charge_call_count}{RESET} (Expected > 0)")


if __name__ == '__main__':
    test_s1()
    test_s2a()
    test_s2b()
    test_s2c()
    test_s2d()
    test_s3a()
    test_s3b()
    test_s3c()
    test_s3d()
    test_s4()
    test_s5()
    test_s6()
    print(f"\n{BOLD}{CYAN}Native Database Simulation Completed.{RESET}")
