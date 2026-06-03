import ast
import sys

filepath = 'fleet_management/FmTrafficHandler.py'
with open(filepath, 'r') as f:
    source = f.read()

try:
    tree = ast.parse(source)
except Exception as e:
    print("Error parsing", e)
    sys.exit(1)

replacements = []

class Visitor(ast.NodeVisitor):
    def visit_Assign(self, node):
        # find: log = self.task_handler.visualization_handler.terminal_log_visualization
        if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'log':
            # We can just remove it
            replacements.append((node.lineno, node.col_offset, node.end_lineno, node.end_col_offset, "pass # log = ... removed"))
        self.generic_visit(node)
        
    def visit_Call(self, node):
        if isinstance(node.func, ast.Name) and node.func.id == 'log':
            # It's a log(...) call
            level = "info"
            if len(node.args) >= 4:
                if isinstance(node.args[3], ast.Constant):
                    level = str(node.args[3].value).lower()
            
            if level not in ('debug', 'info', 'warning', 'error', 'critical'):
                level = 'info'
            
            msg_node = node.args[0]
            msg_src = ast.get_source_segment(source, msg_node)
            
            replacement = f"logger.{level}({msg_src})"
            replacements.append((node.lineno, node.col_offset, node.end_lineno, node.end_col_offset, replacement))
        
        self.generic_visit(node)

Visitor().visit(tree)

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

with open(filepath, 'w') as f:
    f.write("".join(lines))

print("Fixed FmTrafficHandler.py")
