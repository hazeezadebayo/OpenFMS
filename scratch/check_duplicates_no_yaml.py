import re

def find_duplicates(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Simple regex to find loc_id and coordinates in the 'itinerary' section
    # This is a bit fragile but should work for this specific file structure
    itinerary_match = re.search(r'itinerary:\n(.*?)(?=\n\w+:|$)', content, re.DOTALL)
    if not itinerary_match:
        return []
    
    itinerary_content = itinerary_match.group(1)
    
    # Split into individual entries
    entries = re.split(r'\n-\s+', itinerary_content)
    
    locations = {}
    duplicates = []

    for entry in entries:
        if not entry.strip(): continue
        
        # Extract loc_id
        loc_id_match = re.search(r'loc_id:\s+(\w+)', entry)
        if not loc_id_match: continue
        loc_id = loc_id_match.group(1)
        
        # Extract coordinate (first two numbers)
        coord_match = re.search(r'coordinate:\n\s+-\s+([-.\d]+)\n\s+-\s+([-.\d]+)', entry)
        if not coord_match: continue
        
        coords = (float(coord_match.group(1)), float(coord_match.group(2)))
        
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
