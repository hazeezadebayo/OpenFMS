import ast
import os
import sys

def process_file(filepath):
    if not os.path.exists(filepath):
        return

    with open(filepath, 'r') as f:
        source = f.read()
    
    # We will process replacements from bottom to top so that offsets don't change!
    try:
        tree = ast.parse(source)
    except Exception as e:
        print(f"Error parsing {filepath}: {e}")
        return

    replacements = []

    class Visitor(ast.NodeVisitor):
        def visit_Call(self, node):
            if isinstance(node.func, ast.Attribute) and node.func.attr == 'terminal_log_visualization':
                # Determine log level
                level = "info"
                if len(node.args) >= 4:
                    if isinstance(node.args[3], ast.Constant):
                        level = str(node.args[3].value).lower()
                for kw in node.keywords:
                    if kw.arg in ('log_type', 'level') and isinstance(kw.value, ast.Constant):
                        level = str(kw.value.value).lower()
                
                if level not in ('debug', 'info', 'warning', 'error', 'critical'):
                    level = 'info'

                # we need to get the source of the first argument
                msg_node = node.args[0]
                msg_src = ast.get_source_segment(source, msg_node)

                # The replacement string:
                replacement = f"logger.{level}({msg_src})"
                
                # node limits: node.lineno, node.col_offset, node.end_lineno, node.end_col_offset
                replacements.append((node.lineno, node.col_offset, node.end_lineno, node.end_col_offset, replacement))
            
            self.generic_visit(node)

    Visitor().visit(tree)

    # Sort replacements by start position, descending
    replacements.sort(key=lambda x: (x[0], x[1]), reverse=True)

    lines = source.splitlines(keepends=True)

    for start_line, start_col, end_line, end_col, replacement in replacements:
        start_idx = start_line - 1
        end_idx = end_line - 1
        
        if start_idx == end_idx:
            line = lines[start_idx]
            lines[start_idx] = line[:start_col] + replacement + line[end_col:]
        else:
            first_line = lines[start_idx]
            last_line = lines[end_idx]
            
            lines[start_idx] = first_line[:start_col] + replacement + last_line[end_col:]
            for i in range(start_idx + 1, end_idx + 1):
                lines[i] = ""

    new_source = "".join(lines)
    
    # Add logger import if missing
    if "logger = logging.getLogger(__name__)" not in new_source:
        new_source = new_source.replace("import logging", "import logging\nlogger = logging.getLogger(__name__)")
        
    with open(filepath, 'w') as f:
        f.write(new_source)

files = [
    'fleet_management/FmTrafficHandler.py',
    'fleet_management/FmScheduleHandler.py',
    'fleet_management/FmTaskHandler.py',
    'fleet_management/FmMain.py'
]

for f in files:
    print(f"Processing {f}")
    process_file(f)
