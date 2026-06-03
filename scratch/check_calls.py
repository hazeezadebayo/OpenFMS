import ast
import sys

class CallChecker(ast.NodeVisitor):
    def __init__(self):
        self.functions = {}
        self.calls = []

    def visit_FunctionDef(self, node):
        args = [arg.arg for arg in node.args.args]
        defaults_count = len(node.args.defaults)
        min_args = len(args) - defaults_count
        max_args = len(args)
        
        has_vararg = node.args.vararg is not None
        has_kwarg = node.args.kwarg is not None
        
        # Ignore self
        if args and args[0] == 'self':
            min_args = max(0, min_args - 1)
            max_args -= 1
            
        self.functions[node.name] = {
            'min': min_args,
            'max': float('inf') if has_vararg else max_args,
            'has_vararg': has_vararg,
            'has_kwarg': has_kwarg
        }
        self.generic_visit(node)

    def visit_Call(self, node):
        self.generic_visit(node)
        func_name = None
        if isinstance(node.func, ast.Name):
            func_name = node.func.id
        elif isinstance(node.func, ast.Attribute):
            func_name = node.func.attr
        
        if func_name:
            self.calls.append((node.lineno, func_name, len(node.args), len(node.keywords)))

with open(sys.argv[1]) as f:
    tree = ast.parse(f.read())
checker = CallChecker()
checker.visit(tree)

print("Checking function calls against definitions...")
errors = 0
for lineno, func_name, arg_count, kw_count in checker.calls:
    if func_name in checker.functions:
        info = checker.functions[func_name]
        total_args = arg_count + kw_count
        
        if total_args < info['min']:
            print(f"Line {lineno}: '{func_name}' expects at least {info['min']} arguments, got {total_args}.")
            errors += 1
        elif total_args > info['max']:
            print(f"Line {lineno}: '{func_name}' expects at most {info['max']} arguments, got {total_args}.")
            errors += 1

if errors == 0:
    print("All function calls match their definitions perfectly.")
