#!/usr/bin/env python3

from fleet_audit import FleetAuditTool
import json
from datetime import datetime

def analyze_timeline():
    audit = FleetAuditTool()
    conn = audit.connect()
    cursor = conn.cursor()

    print("\n=== Timeline Analysis: The Great Freeze ===")
    
    # 1. When did the first error appear?
    query = """
        SELECT serial_number, timestamp, last_node_id, errors
        FROM state
        WHERE errors != '[]' AND errors != '{}'
        ORDER BY timestamp ASC
        LIMIT 20
    """
    cursor.execute(query)
    rows = cursor.fetchall()

    if not rows:
        print("No error states found in DB.")
        return

    print(f"\nFirst 20 Error Events:")
    print(f"{'Time':<25} | {'Robot':<10} | {'Node':<10} | {'Errors'}")
    print("-" * 75)
    for sn, ts, node, errs in rows:
        if isinstance(errs, str):
            errs = json.loads(errs)
        err_types = [e.get('errorType') for e in errs]
        print(f"{str(ts):<25} | {sn:<10} | {node:<10} | {', '.join(err_types)}")

    # 2. Correlate by Node (Is there a 'deadly' node?)
    query = """
        SELECT last_node_id, COUNT(*) as fail_count
        FROM state
        WHERE errors != '[]' AND errors != '{}'
        GROUP BY last_node_id
        ORDER BY fail_count DESC
    """
    cursor.execute(query)
    node_stats = cursor.fetchall()
    
    print(f"\nFailures by Node Location:")
    for node, count in node_stats:
        print(f"Node {node:<10}: {count} failure reports")

    # 3. Check for order stalling
    print("\nStalled Orders (Released but not reached):")
    orders = audit.get_latest_orders()
    for order in orders:
        sn = order['serial_number']
        nodes = order['nodes']
        if isinstance(nodes, str): nodes = json.loads(nodes)
        
        state = audit.get_robot_state(sn)
        if not state: continue
        
        current_node = state['last_node_id']
        released = [n['nodeId'] for n in nodes if n.get('released')]
        
        if current_node in released:
            idx = released.index(current_node)
            ghosts = released[:idx]
            if ghosts:
                print(f"Robot {sn}: Currently at {current_node}, but STILL RESERVING passed nodes: {ghosts}")

if __name__ == "__main__":
    analyze_timeline()
