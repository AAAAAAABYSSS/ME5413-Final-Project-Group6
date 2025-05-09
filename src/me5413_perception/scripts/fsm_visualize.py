from transitions.extensions import GraphMachine
import os

class Robot:
    pass

robot = Robot()

states = [
    'idle',
    'waiting_for_initial_bbox',
    'navigating_to_furthest_box',
    'evaluating_bridge_conditions',
    'bridge_goal_sent',
    'waiting_at_bridge',
    'crossing_bridge',
    'navigating_to_post_bridge_goal',
    'post_bridge_direct_box_found',
    'post_bridge_scanning',
    'post_bridge_rarest_box_fallback',
    'fallback_navigation',
]

transitions = [
    {'trigger': 'start', 'source': 'idle', 'dest': 'waiting_for_initial_bbox'},
    {'trigger': 'received_bbox', 'source': 'waiting_for_initial_bbox', 'dest': 'navigating_to_furthest_box'},
    {'trigger': 'no_box_available', 'source': 'waiting_for_initial_bbox', 'dest': 'fallback_navigation'},
    {'trigger': 'check_bridge_ready', 'source': 'navigating_to_furthest_box', 'dest': 'evaluating_bridge_conditions'},
    {'trigger': 'send_bridge_goal', 'source': 'evaluating_bridge_conditions', 'dest': 'bridge_goal_sent'},
    {'trigger': 'arrived_at_bridge', 'source': 'bridge_goal_sent', 'dest': 'waiting_at_bridge'},
    {'trigger': 'start_crossing', 'source': 'waiting_at_bridge', 'dest': 'crossing_bridge'},
    {'trigger': 'reached_post_bridge', 'source': 'crossing_bridge', 'dest': 'navigating_to_post_bridge_goal'},
    {'trigger': 'post_bridge_target_found', 'source': 'navigating_to_post_bridge_goal', 'dest': 'post_bridge_direct_box_found'},
    {'trigger': 'start_scanning', 'source': 'navigating_to_post_bridge_goal', 'dest': 'post_bridge_scanning'},
    {'trigger': 'fallback_to_rarest', 'source': 'post_bridge_scanning', 'dest': 'post_bridge_rarest_box_fallback'},
    {'trigger': 'fallback_available', 'source': '*', 'dest': 'fallback_navigation'},
    {'trigger': 'new_box_detected', 'source': 'fallback_navigation', 'dest': 'navigating_to_furthest_box'},
    {'trigger': 'reset', 'source': '*', 'dest': 'idle'}
]

machine = GraphMachine(model=robot, states=states, transitions=transitions, initial='idle', auto_transitions=False)
graph = machine.get_graph()
graph.draw('fsm_diagram.png', prog='dot')
print("Diagram saved as fsm_diagram.png")
