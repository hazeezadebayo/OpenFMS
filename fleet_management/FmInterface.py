#!/usr/bin/env python3

import sys
import os
import time
import random
import argparse
import math

print("[FmInterface] Starting imports...")

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Append the path to the directory containing the file you want to import
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'submodules')))

from FmSimGenerator import GridFleetGraph

print("[FmInterface] Imports complete.")


# ────────────────────────────────────────────────
# S C E N A R I O   D E F I N I T I O N S
# ────────────────────────────────────────────────
#  ['C10', 'C11', 'C3'] Store list of lists e.g. [['C1', 'C2'], ['C10', 'C11']]
SCENARIOS = {
    "S1": {
        "description": "C10 - C11 - C3: [Mutex Group Conflict], C12 - C13: [Only one has free C_node]",
        "custom_chains": [
            "[C2(H) - C10(C) - C11(C) - C3(S)]",
            "[C9(H) - C10(C) - C12(C) - C13(S)]",
            "[C12(C) - W12(W)]"
        ],
        "mutex_groups": [['C10', 'C11', 'C3'],], # Store list of lists
        "num_robots": 2,
        "tasks": [
            {"delay": 2, "robot_id": "R01", "from": "C3", "to": "C13", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 8, "robot_id": "R02", "from": "C13", "to": "C3", "type": "transport", "priority": "low",  "payload": 5, "sent": False}
        ]
    },
    "S2": {
        "description": "C12 - C13: [No waitpoints, both has free C_nodes], [Only one has free C_node], [Both have waitpoints].",
        "custom_chains": [
            "[C10(H) - C11(C) - C12(C) - C13(C) - C14(S)]",
            "[C9(H) - C11(C) - C15(C) - C16(S)]",
            "[C12(C) - C17(C) - C18(S)]",
            "[C13(C) - C19(C)]"
        ],
        "num_robots": 2,
        "tasks": [
            {"delay": 2, "robot_id": "R01", "from": "C14", "to": "C16", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 8, "robot_id": "R02", "from": "C14", "to": "C16", "type": "transport", "priority": "low", "payload": 5, "sent": False}
        ]
    },
    "S3": {
        "description": "No-swap Conflict  – [charge tasks]",
        "custom_chains": [
            "[C10(H) - C11(C) - C12(CH)]",
            "[C9(CH) - C11(C) - C13(H)]"
        ],
        "num_robots": 2,
        "tasks": [
            {"delay": 2, "robot_id": "R01", "from": "C12", "to": "C12", "type": "charge", "priority": "high", "payload": 5, "sent": False},
            {"delay": 8, "robot_id": "R02", "from": "C9", "to": "C9", "type": "charge", "priority": "low", "payload": 5, "sent": False}
        ]
    },
    "S4": {
        "description": "C1-C5 / C2-C6 / C3-C7 / C4-C8: [No waitpoints on main line, both has free C_nodes], [Only one has free C_node with waitpoint option], [Both have waitpoints available]. Multiple parallel lines with cross-connections at C6/C7/C14/C15/C18/C19. Endpoints include station/charge/home docks.",
        "custom_chains": [
            # --- ---
            "[C25(H) - C1(C) - C5(C)]",
            "[C1(C) - W1(W)]",
            "[C5(C) - W5(W)]",
            # # --- ---
            "[C28(H) - C4(C) - C8(C)]",
            "[C4(C) - W4(W)]",
            "[C8(C) - W8(W)]",      
            # # | |
            "[C9(S) - C1(C) - C2(C) - C3(C) - C4(C) - C10(S)]", 
            "[C11(S) - C5(C) - C6(C) - C7(C) - C8(C) - C12(S)]",       
            # # | | 
            "[C21(S) - C13(C) - C14(C) - C15(C) - C16(C) - C22(S)]",
            "[C23(S) - C17(C) - C18(C) - C19(C) - C20(C) - C24(S)]",
            # # --- --- 
            "[C26(H) - C2(C) - C6(C) - C14(C) - C18(C) - C29(H)]",       
            "[C2(C) - W2(W)]",
            "[C6(C) - W6(W)]",
            "[C14(C) - W14(W)]",
            "[C18(C) - W18(W)]",
            # # --- ---  
            "[C27(H) - C3(C) - C7(C) - C15(C) - C19(C) - C30(H)]",        
            "[C3(C) - W3(W)]",
            "[C7(C) - W7(W)]",
            "[C15(C) - W15(W)]",
            "[C19(C) - W19(W)]",
            # # --- ---  
            "[C31(H) - C17(C) - C13(C)]",
            "[C17(C) - W17(W)]",
            "[C13(C) - W13(W)]",
            # # --- ---  
            "[C32(H) - C20(C) - C16(C)]",
            "[C20(C) - W20(W)]",
            "[C16(C) - W16(W)]",
            # # --- ---  
        ],
        "num_robots": 8,
        "tasks": [
            {"delay": 2,  "robot_id": "R01", "home": "C25", "from": "C11", "to": "C12", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 8,  "robot_id": "R02", "home": "C26", "from": "C12", "to": "C11", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            {"delay": 14, "robot_id": "R03", "home": "C27", "from": "C9",  "to": "C10", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 20, "robot_id": "R04", "home": "C28", "from": "C10", "to": "C9",  "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            {"delay": 26, "robot_id": "R05", "home": "C29", "from": "C21", "to": "C22", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 32, "robot_id": "R06", "home": "C30", "from": "C22", "to": "C21", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            {"delay": 38, "robot_id": "R07", "home": "C31", "from": "C23", "to": "C24", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 44, "robot_id": "R08", "home": "C32", "from": "C24", "to": "C23", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
        ]
    },

    "S5": {
        "description": "C1-C5 / C2-C6 / C3-C7 / C4-C8: [No waitpoints on main line, both has free C_nodes], [Only one has free C_node with waitpoint option], [Both have waitpoints available]. Multiple parallel lines with cross-connections at C6/C7/C14/C15/C18/C19. Endpoints include station/charge/home docks.",
        "custom_chains": [
            # --- ---

            # --- GROUP 1 ---

            # --- ---
            "[C25(H) - C1(C) - C5(C)]",
            "[C1(C) - W1(W)]",
            "[C5(C) - W5(W)]",
            # # --- ---
            "[C28(H) - C4(C) - C8(C)]",
            "[C4(C) - W4(W)]",
            "[C8(C) - W8(W)]",      
            # # | |
            "[C9(S) - C1(C) - C2(C) - C3(C) - C4(C) - C10(S)]", 
            "[C11(S) - C5(C) - C6(C) - C7(C) - C8(C) - C12(S)]",       
            # # | | 
            "[C21(S) - C13(C) - C14(C) - C15(C) - C16(C) - C22(S)]",
            "[C23(S) - C17(C) - C18(C) - C19(C) - C20(C) - C24(S)]",
            # # --- --- 
            "[C26(H) - C2(C) - C6(C) - C129(C) - C14(C) - C18(C) - C29(H)]",       
            "[C2(C) - W2(W)]",
            "[C6(C) - W6(W)]",
            "[C14(C) - W14(W)]",
            "[C18(C) - W18(W)]",
            "[C129(C) - W129(W)]",
            # # --- ---  
            "[C27(H) - C3(C) - C7(C) - C130(C) - C15(C) - C19(C) - C30(H)]",        
            "[C3(C) - W3(W)]",
            "[C7(C) - W7(W)]",
            "[C15(C) - W15(W)]",
            "[C19(C) - W19(W)]",
            "[C130(C) - W130(W)]",
            # # --- ---  
            "[C31(H) - C17(C) - C13(C)]",
            "[C17(C) - W17(W)]",
            "[C13(C) - W13(W)]",
            # # --- ---  
            "[C32(H) - C20(C) - C16(C)]",
            "[C20(C) - W20(W)]",
            "[C16(C) - W16(W)]",
            # --- ---  

            # --- GROUP 2 ---

            # --- ---
            "[C130(C) - C131(C)]",
            # --- ---
            "[C57(H) - C33(C) - C37(C)]",
            "[C33(C) - W33(W)]",
            "[C37(C) - W37(W)]",
            # # --- ---
            "[C60(H) - C36(C) - C40(C)]",
            "[C36(C) - W36(W)]",
            "[C40(C) - W40(W)]",      
            # # | |
            "[C41(S) - C33(C) - C34(C) - C35(C) - C36(C) - C42(S)]", 
            "[C43(S) - C37(C) - C38(C) - C39(C) - C40(C) - C44(S)]",       
            # # | | 
            "[C53(S) - C45(C) - C46(C) - C47(C) - C48(C) - C54(S)]",
            "[C55(S) - C49(C) - C50(C) - C51(C) - C52(C) - C56(S)]",
            # # --- --- 
            "[C58(H) - C34(C) - C38(C) - C131(C) - C46(C) - C50(C) - C61(H)]",       
            "[C34(C) - W34(W)]",
            "[C38(C) - W38(W)]",
            "[C46(C) - W46(W)]",
            "[C50(C) - W50(W)]",
            "[C131(C) - W131(W)]",
            # # --- ---  
            "[C59(H) - C35(C) - C39(C) - C132(C) - C47(C) - C51(C) - C62(H)]",        
            "[C35(C) - W35(W)]",
            "[C39(C) - W39(W)]",
            "[C47(C) - W47(W)]",
            "[C51(C) - W51(W)]",
            "[C132(C) - W132(W)]",
            # # --- ---  
            "[C63(H) - C49(C) - C45(C)]",
            "[C49(C) - W49(W)]",
            "[C45(C) - W45(W)]",
            # # --- ---  
            "[C64(H) - C52(C) - C48(C)]",
            "[C52(C) - W52(W)]",
            "[C48(C) - W48(W)]",
            # --- ---

            # --- GROUP 3 ---

            # --- ---
            "[C132(C) - C133(C)]",
            # --- ---
            "[C89(H) - C65(C) - C69(C)]",
            "[C65(C) - W65(W)]",
            "[C69(C) - W69(W)]",
            # --- ---
            "[C92(H) - C68(C) - C72(C)]",
            "[C68(C) - W68(W)]",
            "[C72(C) - W72(W)]",
            # | |
            "[C73(S) - C65(C) - C66(C) - C67(C) - C68(C) - C74(S)]",
            "[C75(S) - C69(C) - C70(C) - C71(C) - C72(C) - C76(S)]",
            # | |
            "[C85(S) - C77(C) - C78(C) - C79(C) - C80(C) - C86(S)]",
            "[C87(S) - C81(C) - C82(C) - C83(C) - C84(C) - C88(S)]",
            # --- ---
            "[C90(H) - C66(C) - C70(C) - C133(C) - C78(C) - C82(C) - C93(H)]",
            "[C66(C) - W66(W)]",
            "[C70(C) - W70(W)]",
            "[C78(C) - W78(W)]",
            "[C82(C) - W82(W)]",
            "[C133(C) - W133(W)]",
            # --- ---
            "[C91(H) - C67(C) - C71(C) - C134(C) - C79(C) - C83(C) - C94(H)]",
            "[C67(C) - W67(W)]",
            "[C71(C) - W71(W)]",
            "[C79(C) - W79(W)]",
            "[C83(C) - W83(W)]",
            "[C134(C) - W134(W)]",
            # --- ---
            "[C95(H) - C81(C) - C77(C)]",
            "[C81(C) - W81(W)]",
            "[C77(C) - W77(W)]",
            # --- ---
            "[C96(H) - C84(C) - C80(C)]",
            "[C84(C) - W84(W)]",
            "[C80(C) - W80(W)]",
            # --- ---

            # --- GROUP 4 ---

            # --- ---
            "[C134(C) - C135(C)]",
            # --- ---
            "[C121(H) - C97(C) - C101(C)]",
            "[C97(C) - W97(W)]",
            "[C101(C) - W101(W)]",
            # --- ---
            "[C124(H) - C100(C) - C104(C)]",
            "[C100(C) - W100(W)]",
            "[C104(C) - W104(W)]",
            # | |
            "[C105(S) - C97(C) - C98(C) - C99(C) - C100(C) - C106(S)]",
            "[C107(S) - C101(C) - C102(C) - C103(C) - C104(C) - C108(S)]",
            # | |
            "[C117(S) - C109(C) - C110(C) - C111(C) - C112(C) - C118(S)]",
            "[C119(S) - C113(C) - C114(C) - C115(C) - C116(C) - C120(S)]",
            # --- ---
            "[C122(H) - C98(C) - C102(C) - C135(C) - C110(C) - C114(C) - C125(H)]",
            "[C98(C) - W98(W)]",
            "[C102(C) - W102(W)]",
            "[C110(C) - W110(W)]",
            "[C114(C) - W114(W)]",
            "[C135(C) - W135(W)]",
            # --- ---
            "[C123(H) - C99(C) - C103(C) - C136(C) - C111(C) - C115(C) - C126(H)]",
            "[C99(C) - W99(W)]",
            "[C103(C) - W103(W)]",
            "[C111(C) - W111(W)]",
            "[C115(C) - W115(W)]",
            "[C136(C) - W136(W)]",
            # --- ---
            "[C127(H) - C113(C) - C109(C)]",
            "[C113(C) - W113(W)]",
            "[C109(C) - W109(W)]",
            # --- ---
            "[C128(H) - C116(C) - C112(C)]",
            "[C116(C) - W116(W)]",
            "[C112(C) - W112(W)]",
            # --- ---
            # --- ---
            # --- ---
        ],
        "num_robots": 2,
        "tasks": [
            # GROUP 1
            {"delay": 2,  "robot_id": "R01", "home": "C25", "from": "C11", "to": "C12", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 5,  "robot_id": "R02", "home": "C28", "from": "C12", "to": "C11", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 8,  "robot_id": "R03", "home": "C26", "from": "C9",  "to": "C10", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 11, "robot_id": "R04", "home": "C27", "from": "C10", "to": "C9",  "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            # {"delay": 14, "robot_id": "R05", "home": "C29", "from": "C21", "to": "C22", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 17, "robot_id": "R06", "home": "C31", "from": "C22", "to": "C21", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 20, "robot_id": "R07", "home": "C30", "from": "C23", "to": "C24", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 23, "robot_id": "R08", "home": "C32", "from": "C24", "to": "C23", "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            # # GROUP 2
            # {"delay": 26, "robot_id": "R09", "home": "C57", "from": "C43", "to": "C44", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 29, "robot_id": "R10", "home": "C60", "from": "C44", "to": "C43", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 32, "robot_id": "R11", "home": "C58", "from": "C41", "to": "C42", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 35, "robot_id": "R12", "home": "C59", "from": "C42", "to": "C41", "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            # {"delay": 38, "robot_id": "R13", "home": "C63", "from": "C53", "to": "C54", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 41, "robot_id": "R14", "home": "C61", "from": "C54", "to": "C53", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 44, "robot_id": "R15", "home": "C62", "from": "C55", "to": "C56", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 47, "robot_id": "R16", "home": "C64", "from": "C56", "to": "C55", "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            # # GROUP 3
            # {"delay": 60, "robot_id": "R17", "home": "C89", "from": "C75", "to": "C76", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 63, "robot_id": "R18", "home": "C92", "from": "C76", "to": "C75", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 66, "robot_id": "R19", "home": "C90", "from": "C73", "to": "C74", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 69, "robot_id": "R20", "home": "C91", "from": "C74", "to": "C73", "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            # {"delay": 72, "robot_id": "R21", "home": "C95", "from": "C85", "to": "C86", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 75, "robot_id": "R22", "home": "C93", "from": "C86", "to": "C85", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 78, "robot_id": "R23", "home": "C94", "from": "C87", "to": "C88", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 81, "robot_id": "R24", "home": "C96", "from": "C88", "to": "C87", "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            # # GROUP 4           
            # {"delay": 84,  "robot_id": "R25", "home": "C121", "from": "C107", "to": "C108", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 87,  "robot_id": "R26", "home": "C124", "from": "C108", "to": "C107", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 90,  "robot_id": "R27", "home": "C122", "from": "C105", "to": "C106", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 93,  "robot_id": "R28", "home": "C123", "from": "C106", "to": "C105", "type": "transport", "priority": "low",  "payload": 5, "sent": False},

            # {"delay": 96,  "robot_id": "R29", "home": "C127", "from": "C117", "to": "C118", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 99,  "robot_id": "R30", "home": "C125", "from": "C118", "to": "C117", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
            # {"delay": 102, "robot_id": "R31", "home": "C126", "from": "C119", "to": "C120", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            # {"delay": 105, "robot_id": "R32", "home": "C128", "from": "C120", "to": "C119", "type": "transport", "priority": "low",  "payload": 5, "sent": False},
        ]
    },
}


# ────────────────────────────────────────────────
# S C E N A R I O   G E N E R A T I O N 
# ────────────────────────────────────────────────

def get_fleet_config(num_robots):
    # Ensure we have a healthy pool of station docks for sampling
    num_station_docks = math.ceil(num_robots * 1.5) + 5
    
    # Standard industrial charging ratio (1 charger per 5 robots)
    num_charge_docks = max(1, math.ceil(num_robots / 5))
    
    # Waitpoints are the 'pockets' for robots to pull into during conflicts
    num_waitpoints = math.ceil(num_robots * 0.5) + 2
    
    # Scale density factor with fleet size to ensure valid graph generation
    if num_robots >= 50:
        density_factor = 20.0
    elif num_robots >= 20:
        density_factor = 8.0
    elif num_robots >= 10:
        density_factor = 3.0
    else:
        density_factor = 1.3
    
    return {
        "num_station_docks": num_station_docks,
        "num_charge_docks": num_charge_docks,
        "num_waitpoints": num_waitpoints,
        "density_factor": density_factor
    }


# ────────────────────────────────────────────────
# M A I N   E X E C U T I O N
# ────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenFMS Live Test Interface")
    parser.add_argument("cmd", choices=["generate", "run"], help="Command to execute")
    parser.add_argument("mode", help="Test scenario to run (e.g. S1, S2, S3) or 'random'.")
    args = parser.parse_args()

    mode = args.mode.upper()
    cmd = args.cmd

    # ==========================================
    # 1. MAP GENERATION PHASE
    # ==========================================
    g = None
    
    if cmd == "generate":
        # ==========================================
        # CASE 1: FIXED SCENARIOS (S1, S2, etc.)
        # ==========================================
        if mode.startswith('S') and mode in SCENARIOS:
            print(f"--- Generating Graph for Scenario: {mode} ---")
            scenario = SCENARIOS[mode]
            # Build robot_id → home_node_id map from tasks that declare a home.
            # Robots without a home entry are not included (FmSimGenerator falls
            # back to sequential assignment for those).
            robot_home_map = {
                t["robot_id"]: t["home"]
                for t in scenario["tasks"]
                if "home" in t
            }
            g = GridFleetGraph(
                num_robots=scenario["num_robots"],
                num_station_docks=0,
                num_charge_docks=0,
                num_waitpoints=0,
                density_factor=0.0,
                custom_chains=scenario["custom_chains"],
                robot_home_map=robot_home_map if robot_home_map else None,
            )         
        # ==========================================
        # CASE 2: PROCEDURAL SCALING (N10, N50, etc.)
        # ==========================================
        elif mode.startswith('N'):
            try:
                # Extract the integer after 'N'
                num_robots = int(mode[1:]) 
                print(f"--- Generating Procedural Graph for {num_robots} Robots ---")
                
                # Fetch the 'Reasonable' configuration
                config = get_fleet_config(num_robots)

                attempt = 1
                while g is None:
                    try:
                        g = GridFleetGraph(
                            num_robots=num_robots, # 3,
                            num_station_docks=config["num_station_docks"], # 6,
                            num_charge_docks=config["num_charge_docks"], # 2,
                            num_waitpoints=config["num_waitpoints"], # 4,
                            density_factor=config["density_factor"] # 1.3,
                        ) 
                    except RuntimeError as e:
                        print(f"[Attempt {attempt}] Grid generation failed ({e}). Retrying...")
                        attempt += 1
                        if attempt > 20: 
                            print("CRITICAL: Could not generate valid graph. Adjust density.")
                            sys.exit(1)
            except ValueError:
                print(f"❌ Error: Invalid format '{mode}'. Use N followed by a number (e.g., N25).")
                sys.exit(1)
        else:
            print(f"❌ Error: Unknown mode '{mode}'. Use S<number> for scenarios or N<number> for scaling.")
            sys.exit(1)

        print(f"\n✅ Map generation complete. Exiting.")
        if g is not None:
            g.plot(f"{mode}_grid_layout.png")

        # generation phase completed.
        sys.exit(0)

    # ==========================================
    # 2. FLEET MANAGER RUN PHASE
    # ==========================================

    from FmMain import FmMain
    
    print(f"\n--- Initializing Fleet Manager for Scenario: {mode} ---")
    fm = FmMain()
    
    # Apply mutex groups if defined in scenario
    if mode in SCENARIOS and "mutex_groups" in SCENARIOS[mode]:
        print(f"Applying Mutex Groups: {SCENARIOS[mode]['mutex_groups']}")
        fm.schedule_handler.traffic_handler.mutex_groups = SCENARIOS[mode]["mutex_groups"]

    # FmInterface is the source: we own the single thread.
    # DO NOT start a background loop — run_cycle() is called below inside
    # the scenario loop so robot-management and task-dispatch share one thread.
    time.sleep(5)  # brief warmup for MQTT/state to populate

    start_time = time.time()
    task_trigger_time = 5 
    
    if mode in SCENARIOS:
        tasks = SCENARIOS[mode]["tasks"]
        print(f"Goal: {SCENARIOS[mode]['description']}")

    else:
        # 1. Parse robot count from mode (e.g., N10 -> 10)
        # We exit strictly if the format is incorrect to prevent undefined behavior.
        try:
            if not mode.startswith('N'):
                raise ValueError("Random mode must start with 'N' followed by a number.")
            num_robots = int(mode[1:])
        except (ValueError, IndexError) as e:
            print(f"❌ Critical Error: Could not evaluate robot count from '{mode}'.")
            print(f"Details: {e}")
            sys.exit(1)

        # 2. Prepare remaining docks for task randomization
        itinerary = fm.task_dictionary.get('itinerary', [])
        station_docks = [n['loc_id'] for n in itinerary if n.get('description') == "station_dock"]
        charge_docks = [n['loc_id'] for n in itinerary if n.get('description') == "charge_dock"]

        # Uniqueness guarantee: shuffle and pop
        random.shuffle(station_docks)
        random.shuffle(charge_docks)

        priorities = ["high", "medium", "low"]
        tasks = []

        # 1. Determine the necessary padding length
        # log10 or len(str()) both work; len(str()) is more "Pythonic"
        padding_limit = max(2, len(str(num_robots))) 

        for i in range(num_robots):
            # 2. Generate ID dynamically: R01, R02.. R100... R0001, R0002, etc.
            robot_id = f"R{str(i+1).zfill(padding_limit)}"
            
            # Randomize priority for this specific robot's task
            current_priority = random.choice(priorities)
            
            # 3. Task Selection Logic
            # Set to a threshold above our current test limit (N50) to ensure purely transport tasks
            if i >= 60 and i % 5 == 0 and len(charge_docks) > 0:
                # CHARGE TASK: from == to
                dock = charge_docks.pop(0)
                tasks.append({
                    "delay": i * 5,
                    "robot_id": robot_id,
                    "from": dock,
                    "to": dock,
                    "type": "charge",
                    "priority": current_priority,
                    "payload": 0,
                    "sent": False
                })
                print(f"[Task Gen] {robot_id} ({current_priority.upper()}): Charging at {dock}")
            
            elif len(station_docks) >= 2:
                # # TRANSPORT TASK: Unique Start and unique End
                # start_node = station_docks.pop(0)
                # end_node = station_docks.pop(0)
                # TRANSPORT TASK: Unique Start and unique End for this specific task
                # Using random.sample instead of pop(0) to support high-N without dock depletion
                start_node, end_node = random.sample(station_docks, 2)
                tasks.append({
                    "delay": i * 5,
                    "robot_id": robot_id,
                    "from": start_node,
                    "to": end_node,
                    "type": "transport",
                    "priority": current_priority,
                    "payload": 5,
                    "sent": False
                })
                print(f"[Task Gen] {robot_id} ({current_priority.upper()}): {start_node} -> {end_node}")
            
            else:
                print(f"⚠️ Warning: Insufficient docks for {robot_id}. Task generation halted.")
                break

        print(f"\n✅ Fleet Configuration Ready: {len(tasks)} robots initialized with mixed priorities.")

    # Warmup: request factsheets from all robots so the FM has robot data
    # set the fleet_ids and robot_ids
    fm.schedule_handler.fm_send_factsheet_request(fm.manufacturer, fm.version)
    fm.fleetnames = fm.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_fleets()
    fm.fleetname = "kullar" if "kullar" in fm.fleetnames else None
    fm.upload_all_maps(fm.fleetname)
    fm.job_ids = fm.process_itinerary(fm.task_dictionary.get("itinerary", []), fm.fleetname)
    fm.serial_numbers = fm.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_serial_numbers(fm.fleetname)
    
    # # Categorize docks for event monitoring (simulation completion and station arrivals)
    # itinerary = fm.task_dictionary.get('itinerary', [])
    # home_docks = {n['loc_id'] for n in itinerary if n.get('description') == "home_dock"}
    # all_station_docks = {n['loc_id'] for n in itinerary if n.get('description') == "station_dock"}

    try:
        while True:
            elapsed_time = time.time() - start_time
            if int(elapsed_time) % 5 == 0:
                print(f"\r[Elapsed Time: {int(elapsed_time)}s]", end="", flush=True)

            for t in tasks:
                if elapsed_time >= task_trigger_time + t["delay"] and not t["sent"]:
                    print(f"\n[Time: {int(elapsed_time)}s] Dispatching {t['type']} task for {t['robot_id']}: {t['from']} -> {t['to']}")
                    fm.fm_dispatch_task(
                        fleet_id="kullar",
                        robot_id=t["robot_id"],
                        from_loc=t["from"],
                        to_loc=t["to"],
                        task_name=t["type"],
                        priority=t["priority"],
                        payload=t["payload"]
                    )
                    t["sent"] = True

            # ── Single-thread fleet management ─────────────────────────────
            # run_cycle() manages all robots for one full pass then returns.
            # This keeps task-dispatch and robot-management on the same thread.
            fm.run_cycle()
            # ───────────────────────────────────────────────────────────────


    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    
    fm.cleanup()
        