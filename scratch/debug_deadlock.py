import psycopg2
import json
import datetime
from typing import List, Dict, Any

class FleetAnalyzer:
    def __init__(self):
        self.conn = None
        self.connect()

    def connect(self):
        hosts = ["db", "localhost"]
        for host in hosts:
            try:
                self.conn = psycopg2.connect(host=host, database="postgres", user="postgres", password="root")
                print(f"Connected to DB on host: {host}")
                return
            except:
                continue
        raise Exception("Could not connect to database.")

    def get_robot_states(self) -> Dict[str, Any]:
        cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
        query = """
            SELECT DISTINCT ON (serial_number) *
            FROM state
            ORDER BY serial_number, timestamp DESC;
        """
        cur.execute(query)
        rows = cur.fetchall()
        return {row['serial_number']: dict(row) for row in rows}

    def get_robot_orders(self) -> Dict[str, Any]:
        cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
        query = """
            SELECT DISTINCT ON (serial_number) *
            FROM orders
            ORDER BY serial_number, timestamp DESC;
        """
        cur.execute(query)
        rows = cur.fetchall()
        return {row['serial_number']: dict(row) for row in rows}

    def analyze(self):
        states = self.get_robot_states()
        orders = self.get_robot_orders()

        print("\n=== Fleet Status Analysis ===\n")
        
        traffic_control = {}
        for sn, order in orders.items():
            if sn.startswith("unassigned_"):
                continue
            nodes = order['nodes']
            released_nodes = [node['nodeId'] for node in nodes if node.get('released')]
            traffic_control[sn] = released_nodes

        print(f"Traffic Control (Reserved Nodes): {traffic_control}\n")

        for sn in sorted(states.keys()):
            state = states[sn]
            order = orders.get(sn)
            
            last_node = state.get('last_node_id')
            pos = state.get('agv_position')
            battery = state.get('battery_state', {}).get('batteryCharge')
            driving = state.get('driving')
            paused = state.get('paused')
            halt = state.get('halt')
            errors = state.get('errors')
            
            print(f"Robot: {sn}")
            print(f"  Position: x={pos.get('x')}, y={pos.get('y')}, theta={pos.get('theta')}")
            print(f"  Last Node: {last_node}")
            print(f"  Battery: {battery}%")
            print(f"  Driving: {driving}, Paused: {paused}, Halt: {halt}")
            if errors:
                print(f"  Errors: {errors}")
            
            if order:
                nodes = order['nodes']
                horizon = [n['nodeId'] for n in nodes if not n.get('released')]
                reserved = [n['nodeId'] for n in nodes if n.get('released')]
                print(f"  Active Order ID: {order['order_id']}")
                print(f"  Reserved Nodes: {reserved}")
                print(f"  Horizon Nodes: {horizon}")
                
                # Check for "running late" logic (simplified)
                # In actual system, it checks eta in nodeDescription
                for node in nodes:
                    desc = node.get('nodeDescription', '')
                    if "Node ETA:" in desc:
                        eta_str = desc.split("Node ETA: ")[-1]
                        if eta_str and eta_str != 'None':
                            try:
                                eta = datetime.datetime.fromisoformat(eta_str)
                                now = datetime.datetime.now()
                                if now > eta:
                                    print(f"  [WARN] Running late for node {node['nodeId']} (ETA: {eta_str})")
                            except:
                                pass

            print("-" * 40)

        # Check for cycles in reservations
        self.detect_deadlock(traffic_control, states, orders)

    def detect_deadlock(self, traffic_control, states, orders):
        print("\n=== Deadlock Detection ===\n")
        
        # A robot X is waiting for node N.
        # Node N is occupied/reserved by robot Y.
        
        waiting_for = {}
        for sn, state in states.items():
            order = orders.get(sn)
            if not order:
                continue
            
            nodes = order['nodes']
            horizon = [n['nodeId'] for n in nodes if not n.get('released')]
            if horizon:
                next_node = horizon[0]
                # Who has next_node?
                occupant = None
                for other_sn, reserved in traffic_control.items():
                    if next_node in reserved:
                        occupant = other_sn
                        break
                
                if occupant:
                    waiting_for[sn] = (next_node, occupant)
                    print(f"Robot {sn} is waiting for {next_node}, which is held by {occupant}")
                else:
                    print(f"Robot {sn} is waiting for {next_node}, but NO ONE seems to hold it in reserved list?")

        # Check for wait cycles
        visited = set()
        for start_node in waiting_for:
            if start_node in visited:
                continue
            
            path = []
            curr = start_node
            while curr in waiting_for and curr not in path:
                path.append(curr)
                visited.add(curr)
                _, next_robot = waiting_for[curr]
                curr = next_robot
            
            if curr in path:
                cycle = path[path.index(curr):]
                print(f"\n!!! DEADLOCK CYCLE DETECTED: {' -> '.join(cycle)} -> {curr} !!!")

if __name__ == "__main__":
    import psycopg2.extras
    analyzer = FleetAnalyzer()
    analyzer.analyze()
