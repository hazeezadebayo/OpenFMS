import ast
import sys

class Checker(ast.NodeVisitor):
    def __init__(self):
        self.scopes = [set()]
        self.globals = set(dir(__builtins__))
        self.globals.update(['math', 'datetime', 'cast', 'List', 'Dict', 'Any', 'Optional', 'os', 'json', 'time', 'uuid', 'Tuple', '__file__'])
        self.errors = []

    def visit_FunctionDef(self, node):
        self.scopes[-1].add(node.name)
        new_scope = set()
        for arg in node.args.args:
            new_scope.add(arg.arg)
        for arg in node.args.kwonlyargs:
            new_scope.add(arg.arg)
        if node.args.vararg:
            new_scope.add(node.args.vararg.arg)
        if node.args.kwarg:
            new_scope.add(node.args.kwarg.arg)
        self.scopes.append(new_scope)
        self.generic_visit(node)
        self.scopes.pop()

    def visit_ClassDef(self, node):
        self.scopes[-1].add(node.name)
        self.scopes.append(set())
        self.generic_visit(node)
        self.scopes.pop()

    def visit_Assign(self, node):
        self.generic_visit(node)
        for target in node.targets:
            if isinstance(target, ast.Name):
                self.scopes[-1].add(target.id)
            elif isinstance(target, ast.Tuple):
                for el in target.elts:
                    if isinstance(el, ast.Name):
                        self.scopes[-1].add(el.id)
                    elif isinstance(el, ast.Starred) and isinstance(el.value, ast.Name):
                        self.scopes[-1].add(el.value.id)

    def visit_AnnAssign(self, node):
        self.generic_visit(node)
        if isinstance(node.target, ast.Name):
            self.scopes[-1].add(node.target.id)

    def visit_For(self, node):
        if isinstance(node.target, ast.Name):
            self.scopes[-1].add(node.target.id)
        elif isinstance(node.target, ast.Tuple):
            for el in node.target.elts:
                if isinstance(el, ast.Name):
                    self.scopes[-1].add(el.id)
        self.generic_visit(node)

    def visit_ExceptHandler(self, node):
        if node.name:
            self.scopes[-1].add(node.name)
        self.generic_visit(node)

    def visit_ListComp(self, node):
        new_scope = set()
        self.scopes.append(new_scope)
        for comp in node.generators:
            if isinstance(comp.target, ast.Name):
                self.scopes[-1].add(comp.target.id)
            elif isinstance(comp.target, ast.Tuple):
                for el in comp.target.elts:
                    if isinstance(el, ast.Name):
                        self.scopes[-1].add(el.id)
        self.generic_visit(node)
        self.scopes.pop()

    def visit_SetComp(self, node):
        self.visit_ListComp(node)

    def visit_GeneratorExp(self, node):
        self.visit_ListComp(node)

    def visit_DictComp(self, node):
        self.visit_ListComp(node)

    def visit_Name(self, node):
        if isinstance(node.ctx, ast.Load):
            found = False
            for scope in reversed(self.scopes):
                if node.id in scope:
                    found = True
                    break
            if not found and node.id not in self.globals:
                self.errors.append((node.lineno, node.id))
        self.generic_visit(node)

    def visit_Import(self, node):
        for alias in node.names:
            name = alias.asname or alias.name
            self.scopes[-1].add(name.split('.')[0])
            
    def visit_ImportFrom(self, node):
        for alias in node.names:
            name = alias.asname or alias.name
            self.scopes[-1].add(name)

with open(sys.argv[1]) as f:
    tree = ast.parse(f.read())
checker = Checker()
checker.visit(tree)
for line, name in sorted(checker.errors):
    print(f"Line {line}: undefined name '{name}'")
