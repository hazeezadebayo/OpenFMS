import re
from collections import defaultdict

def check_all_coordinates(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Find the itinerary section
    itinerary_match = re.search(r'itinerary:\n(.*?)(?=\n\w+:|$)', content, re.DOTALL)
    if not itinerary_match:
        print("Could not find itinerary section")
        return
    
    itinerary_content = itinerary_match.group(1)
    
    # Split into entries
    # Entries start with '- coordinate:'
    entries = re.split(r'\n-\s+coordinate:', itinerary_content)
    
    locations = defaultdict(list)

    for entry in entries:
        if not entry.strip(): continue
        
        # Extract loc_id
        loc_id_match = re.search(r'loc_id:\s+(\w+)', entry)
        if not loc_id_match: continue
        loc_id = loc_id_match.group(1)
        
        # Extract coordinates
        # Coordinates are the first two lines starting with '-' after 'coordinate:'
        # Since we split by '- coordinate:', the entry starts right at the coordinates
        coord_lines = re.findall(r'^\s+-\s+([-.\d]+)', entry, re.MULTILINE)
        if len(coord_lines) < 2: continue
        
        coords = (float(coord_lines[0]), float(coord_lines[1]))
        locations[coords].append(loc_id)

    duplicates = {k: v for k, v in locations.items() if len(v) > 1}
    
    if duplicates:
        print("Found duplicate coordinates:")
        for coords, ids in duplicates.items():
            print(f"Location {coords} is shared by: {', '.join(ids)}")
    else:
        print("SUCCESS: No duplicate coordinates found in itinerary.")
        # Print a sample to verify parsing worked
        print(f"Total unique locations parsed: {len(locations)}")

if __name__ == "__main__":
    path = "/home/azeez/ws/dev_env/py_code/projects/phd/OpenFMS/config/config.yaml"
    check_all_coordinates(path)
