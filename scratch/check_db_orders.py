import psycopg2
import json

def check_orders():
    # Try both 'db' and 'localhost'
    hosts = ["db", "localhost"]
    conn = None
    for host in hosts:
        try:
            conn = psycopg2.connect(host=host, database="postgres", user="postgres", password="root")
            print(f"Connected to DB on host: {host}")
            break
        except:
            continue
    
    if not conn:
        print("Could not connect to database.")
        return

    cur = conn.cursor()
    cur.execute("SELECT order_id, serial_number, nodes FROM orders WHERE order_id LIKE '%_completed';")
    rows = cur.fetchall()
    print(f"Found {len(rows)} completed orders in DB:")
    for order_id, sn, nodes in rows:
        try:
            # nodes might be a string or a list
            if isinstance(nodes, str):
                nodes_data = json.loads(nodes)
            else:
                nodes_data = nodes
                
            task_type = "Unknown"
            if nodes_data and len(nodes_data) > 0:
                # Find the dock action
                for node in nodes_data:
                    actions = node.get("actions", [])
                    for action in actions:
                        if action.get("actionType") == "dock":
                            params = action.get("actionParameters", [{}])[0].get("value", [])
                            if len(params) > 2:
                                task_type = params[2]
                                break
                    if task_type != "Unknown":
                        break
            
            print(f"  >> Matches Transport Signature: ")
            print(f"Order: {order_id} | Robot: {sn} | Task Type: {task_type}")
        except Exception as e:
            print(f"Order: {order_id} | Robot: {sn} | Error parsing nodes: {e}")
    
    cur.close()
    conn.close()

if __name__ == "__main__":
    check_orders()
