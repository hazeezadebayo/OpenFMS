import sys
import os
sys.path.append(os.getcwd())
from submodules.visualization import VisualizationSubscriber

graph = {
  'C1': [['C2', 0.1], ['C25', 0.1], ['C5', 0.1], ['C9', 0.1], ['W1', 0.1]],
  'C2': [['C1', 0.1], ['C26', 0.1], ['C3', 0.1], ['C6', 0.1], ['W2', 0.1]],
  'C3': [['C2', 0.1], ['C27', 0.1], ['C4', 0.1], ['C7', 0.1], ['W3', 0.1]],
  'C7': [['C15', 0.1], ['C3', 0.1], ['C6', 0.1], ['C8', 0.1], ['W7', 0.1]],
  'C8': [['C12', 0.1], ['C4', 0.1], ['C7', 0.1], ['W8', 0.1]],
  'C11': [['C5', 0.1]],
  'C5': [['C1', 0.1], ['C11', 0.1], ['C6', 0.1], ['W5', 0.1]]
}

itinerary = [
  {'loc_id': 'C1', 'coordinate': [6.0, 0.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C2', 'coordinate': [6.0, -6.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C3', 'coordinate': [12.0, -6.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C4', 'coordinate': [18.0, -12.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C5', 'coordinate': [12.0, 0.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C6', 'coordinate': [12.0, 6.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C7', 'coordinate': [18.0, 6.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C8', 'coordinate': [24.0, -12.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C9', 'coordinate': [6.0, 6.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C10', 'coordinate': [18.0, -6.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C11', 'coordinate': [18.0, 0.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C12', 'coordinate': [30.0, -12.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C15', 'coordinate': [48.0, -18.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C25', 'coordinate': [0.0, 0.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C26', 'coordinate': [0.0, -6.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C27', 'coordinate': [6.0, -12.0], 'fleet_id': 'kullar'},
  {'loc_id': 'C28', 'coordinate': [12.0, -12.0], 'fleet_id': 'kullar'},
  {'loc_id': 'W1', 'coordinate': [7.591, 1.591], 'fleet_id': 'kullar'},
  {'loc_id': 'W2', 'coordinate': [7.591, -4.409], 'fleet_id': 'kullar'},
  {'loc_id': 'W3', 'coordinate': [13.591, -4.409], 'fleet_id': 'kullar'},
  {'loc_id': 'W4', 'coordinate': [19.591, -10.409], 'fleet_id': 'kullar'},
  {'loc_id': 'W5', 'coordinate': [13.591, 1.591], 'fleet_id': 'kullar'},
  {'loc_id': 'W7', 'coordinate': [19.591, 7.591], 'fleet_id': 'kullar'},
  {'loc_id': 'W8', 'coordinate': [25.591, -10.409], 'fleet_id': 'kullar'}
]

v = VisualizationSubscriber('kullar', 'v2', None)
v.task_dictionary = {'graph': graph, 'itinerary': itinerary}
v.robot_positions = {}
v.terminal_graph_visualization()
