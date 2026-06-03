import yaml
with open('config/config.yaml') as f:
    config = yaml.safe_load(f)
print("Keys in map_graph:", list(config['map_graph'].keys())[:10])
