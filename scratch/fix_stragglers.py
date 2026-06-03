import re

# FmScheduleHandler.py
with open('fleet_management/FmScheduleHandler.py', 'r') as f:
    c = f.read()
c = re.sub(r'self\.traffic_handler\.task_handler\.visualization_handler\.terminal_log_visualization\(\s*message,\s*"FmScheduleHandler",\s*"fm_analytics",\s*level\s*\)', r'getattr(logger, level if level in ["debug","info","warning","error","critical"] else "info")(message)', c)
with open('fleet_management/FmScheduleHandler.py', 'w') as f:
    f.write(c)

# FmTrafficHandler.py
with open('fleet_management/FmTrafficHandler.py', 'r') as f:
    c = f.read()

# Replace aliases: log = self.task_handler... -> removed, and then replace calls to log(
c = re.sub(r'log = self\.task_handler\.visualization_handler\.terminal_log_visualization\n', '', c)
# E.g. log(f"{r_ctx.robot_id} waiting...", "FmTrafficHandler", "_handle_active_mex_conflict", "info")
# E.g. log(f"however, mex_r_id {mex_ctx.robot_id} is halted...", "FmTrafficHandler", "_handle_active_mex_conflict", "info")
# Convert log(msg, class, func, level) to logger.<level>(msg)
def replacer_log(match):
    msg = match.group(1)
    level = match.group(2)
    if level == 'warn': level = 'warning'
    return f'logger.{level}({msg})'

c = re.sub(r'log\(\s*(.*?),\s*["\']\w+["\']\s*,\s*["\']\w+["\']\s*,\s*["\'](debug|info|warn|error|critical)["\']\s*\)', replacer_log, c, flags=re.DOTALL)

# Handle the regular ones missed because they lacked the exact arg count
c = re.sub(r'self\.task_handler\.visualization_handler\.terminal_log_visualization\(\s*(.*?)\)', r'logger.warning(\1)', c, flags=re.DOTALL)

with open('fleet_management/FmTrafficHandler.py', 'w') as f:
    f.write(c)

print("done")
