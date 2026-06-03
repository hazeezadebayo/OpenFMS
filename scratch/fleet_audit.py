#!/usr/bin/env python3

import psycopg2
import psycopg2.extras
import json
import datetime
from typing import List, Dict, Any, Optional

class FleetAuditTool:
    def __init__(self, host="db", dbname="postgres", user="postgres", password="root", port=5432):
        self.conn_params = {
            "host": host,
            "dbname": dbname,
            "user": user,
            "password": password,
            "port": port
        }
        self.conn = None

    def connect(self):
        if not self.conn or self.conn.closed:
            self.conn = psycopg2.connect(**self.conn_params)
        return self.conn

    def get_latest_orders(self, fleet_id: Optional[str] = None) -> List[Dict[str, Any]]:
        conn = self.connect()
        cursor = conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor)
        
        query = """
            SELECT DISTINCT ON (serial_number) 
                id, header_id, timestamp, serial_number, order_id, order_update_id, nodes, edges
            FROM orders
            WHERE 1=1
        """
        params = []
        if fleet_id:
            query += " AND zone_set_id = %s"
            params.append(fleet_id)
        
        query += " ORDER BY serial_number, timestamp DESC"
        
        cursor.execute(query, params)
        return cursor.fetchall()

    def get_robot_state(self, serial_number: str) -> Optional[Dict[str, Any]]:
        conn = self.connect()
        cursor = conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor)
        
        query = """
            SELECT * FROM state
            WHERE serial_number = %s
            ORDER BY timestamp DESC
            LIMIT 1
        """
        cursor.execute(query, (serial_number,))
        return cursor.fetchone()

    def analyze_deadlock(self):
        print("\n--- Fleet Deadlock Audit ---")
        orders = self.get_latest_orders()
        
        reservations = {}
        robot_positions = {}
        robot_errors = {}

        for order in orders:
            sn = order['serial_number']
            oid = order['order_id']
            
            # Skip completed/cancelled
            if oid.endswith("_completed") or oid.endswith("_cancelled"):
                continue
                
            nodes = order['nodes']
            if isinstance(nodes, str):
                nodes = json.loads(nodes)
            
            state = self.get_robot_state(sn)
            pos = state.get('last_node_id', 'Unknown') if state else 'Unknown'
            robot_positions[sn] = pos

            # Logic to identify ghost reservations
            node_ids = [n['nodeId'] for n in nodes]
            released_nodes = [n['nodeId'] for n in nodes if n.get('released')]
            
            ghost_nodes = []
            if pos in node_ids:
                base_idx = node_ids.index(pos)
                passed_nodes = node_ids[:base_idx]
                ghost_nodes = [n for n in passed_nodes if n in released_nodes]

            reservations[sn] = {
                "active": [n for n in released_nodes if n not in ghost_nodes],
                "ghosts": ghost_nodes
            }
            
            if state:
                errors = state.get('errors', [])
                if isinstance(errors, str):
                    errors = json.loads(errors)
                robot_errors[sn] = errors

        print(f"{'Robot':<10} | {'At Node':<10} | {'Active Res':<25} | {'Ghost Res':<15} | {'Errors'}")
        print("-" * 100)
        for sn, data in reservations.items():
            pos = robot_positions.get(sn, "?")
            active = ", ".join(data["active"])
            ghosts = ", ".join(data["ghosts"])
            errs = ", ".join([e.get('errorType', 'Err') for e in robot_errors.get(sn, [])])
            print(f"{sn:<10} | {pos:<10} | {active:<25} | {ghosts:<15} | {errs}")

        # Cycle Detection
        self._find_cycles(reservations, robot_positions)

    def _find_cycles(self, reservations, robot_positions):
        # Build dependency graph: Robot A waits for Node X, which is held by Robot B
        waiting_for = {}
        for sn, res in reservations.items():
            current_pos = robot_positions.get(sn)
            if not current_pos or current_pos == 'Unknown':
                continue
            
            # If robot is not at its destination (the last node in its released set), 
            # it might be waiting for the NEXT node in some path (complex to determine here).
            # Simplified: Check if any robot is blocked by another's reservation.
            pass

        print("\nNote: Use analyze_freeze.py for detailed timeline analysis.")

if __name__ == "__main__":
    audit = FleetAuditTool()
    audit.analyze_deadlock()
