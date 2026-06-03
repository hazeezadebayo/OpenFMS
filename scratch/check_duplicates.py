import yaml

def find_duplicates(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    
    itinerary = data.get('itinerary', [])
    locations = {}
    duplicates = []

    for entry in itinerary:
        loc_id = entry.get('loc_id')
        coords = tuple(entry.get('coordinate', [])[:2]) # Just X and Y
        
        if coords in locations:
            locations[coords].append(loc_id)
        else:
            locations[coords] = [loc_id]

    for coords, ids in locations.items():
        if len(ids) > 1:
            duplicates.append((coords, ids))
            
    return duplicates

if __name__ == "__main__":
    path = "/home/azeez/ws/dev_env/py_code/projects/phd/OpenFMS/config/config.yaml"
    dupes = find_duplicates(path)
    if dupes:
        print("Found duplicate coordinates:")
        for coords, ids in dupes:
            print(f"Location {coords} is shared by: {', '.join(ids)}")
    else:
        print("No duplicate coordinates found.")
