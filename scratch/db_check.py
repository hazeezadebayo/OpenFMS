import psycopg2
import json

try:
    conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
    cursor = conn.cursor()
    cursor.execute("SELECT timestamp, order_id, nodes FROM orders WHERE serial_number = 'R01' ORDER BY timestamp DESC LIMIT 5;")
    rows = cursor.fetchall()
    for row in rows:
        ts = row[0]
        order_id = row[1]
        nodes = row[2]
        node_ids = [n['nodeId'] for n in sorted(nodes, key=lambda x: x['sequenceId'])]
        print(f"Time: {ts} | OrderID: {order_id} | Nodes: {node_ids}")
except Exception as e:
    print(e)
