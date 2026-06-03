import os
import re

files = [
    'fleet_management/FmTrafficHandler.py',
    'fleet_management/FmScheduleHandler.py',
    'fleet_management/FmTaskHandler.py',
    'fleet_management/FmMain.py'
]

def process_file(filepath):
    if not os.path.exists(filepath): return
    with open(filepath, 'r') as f:
        content = f.read()

    # We need to find all occurrences of `.terminal_log_visualization(`
    # and replace them with `logger.xxx(`.
    # Since arguments can contain parentheses and span multiple lines, we can parse it carefully.
    
    out_content = ""
    idx = 0
    while True:
        match = re.search(r'(?:self\.)?(?:traffic_handler\.)?(?:task_handler\.)?(?:visualization_handler\.)?terminal_log_visualization\(', content[idx:])
        if not match:
            out_content += content[idx:]
            break
            
        start_pos = idx + match.start()
        prefix = content[idx:start_pos]
        out_content += prefix
        
        # Now find the matching closing parenthesis
        open_parens = 1
        inner_start = start_pos + len(match.group(0))
        curr = inner_start
        in_string = False
        escape = False
        string_char = ''
        
        while curr < len(content):
            c = content[curr]
            if escape:
                escape = False
            elif c == '\\':
                escape = True
            elif in_string:
                if c == string_char:
                    # Might be triple quotes, but let's assume simple string bounds or f-strings for now
                    # (This is simplified, might fail on triple quotes, but usually logs are single-line strings)
                    # Let's check for triple quotes
                    if curr >= 2 and content[curr-2:curr+1] == string_char * 3:
                        in_string = False
                    elif content[curr-2:curr+1] != string_char * 3:
                        # Wait, handling triple quotes properly is tricky. 
                        # Let's assume standard strings.
                        in_string = False
            else:
                if c in ('"', "'"):
                    in_string = True
                    string_char = c
                elif c == '(':
                    open_parens += 1
                elif c == ')':
                    open_parens -= 1
                    if open_parens == 0:
                        break
            curr += 1
            
        if curr == len(content):
            print("Unmatched parenthesis!")
            break
            
        args_str = content[inner_start:curr]
        idx = curr + 1 # skip ')'
        
        # Now parse args_str. Usually it's: msg, class, func, level
        # We can just split by comma, BUT commas can be inside strings or parentheses.
        # So we use ast to parse the args if possible, or just string manipulation.
        # Better: just wrap the whole thing in a function and use ast!
        try:
            import ast
            # wrap in a dummy function call
            tree = ast.parse(f"dummy({args_str})")
            call = tree.body[0].value
            msg_node = call.args[0]
            
            # The level is usually the 4th argument (index 3), or kwargs
            level = "debug"
            if len(call.args) >= 4:
                level_node = call.args[3]
                if isinstance(level_node, ast.Constant):
                    level = level_node.value.lower()
                elif isinstance(level_node, ast.Str): # Python < 3.8
                    level = level_node.s.lower()
            for kw in call.keywords:
                if kw.arg in ('log_type', 'level'):
                    if isinstance(kw.value, ast.Constant):
                        level = kw.value.value.lower()
                    elif isinstance(kw.value, ast.Str):
                        level = kw.value.s.lower()
            
            if level not in ('debug', 'info', 'warning', 'error', 'critical'):
                level = 'info'
                
            # Extract the raw string for the first argument (message)
            import astor # if available? No, let's just use ast.get_source_segment if python >= 3.8
            import sys
            if sys.version_info >= (3, 8):
                msg_src = ast.get_source_segment(f"dummy({args_str})", msg_node)
            else:
                msg_src = args_str.split(',')[0].strip() # fallback
                
            out_content += f"logger.{level}({msg_src})"
        except Exception as e:
            print(f"Error parsing args: {args_str}, {e}")
            out_content += f"logger.info({args_str})"

    with open(filepath, 'w') as f:
        f.write(out_content)

for filepath in files:
    print(f"Processing {filepath}")
    process_file(filepath)

