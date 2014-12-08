__author__ = 'kristjan'

from rsd_smach.navigatestates import navigate_base_state


def build_dispenser(task_list, onPreempt):
    waypoint_lists = {'IN_BOX': [[3,0],[3,-3],[0,-3],[0,0]]}
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_DISPENSER', waypoint_lists)


def build_in_box(task_list, onPreempt):
    waypoint_lists = {'RAMP_IN': [[3,0],[3,-3],[0,-3],[0,0]],
                      'STATION_1': [[3,0],[3,-3],[0,-3],[0,0]],
                      'STATION_2': [[3,0],[3,-3],[0,-3],[0,0]],
                      'STATION_3': [[3,0],[3,-3],[0,-3],[0,0]]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_IN_BOX', waypoint_lists)


def build_ramp_out(task_list, onPreempt):
    waypoint_lists = {'DISPENSER': [[3,0],[3,-3],[0,-3],[0,0]],
                      'IN_BOX': [[3,0],[3,-3],[0,-3],[0,0]]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_RAMP_OUT', waypoint_lists)


def build_ramp_in(task_list, onPreempt):
    waypoint_lists = {'FLOOR_IN': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_RAMP_IN', waypoint_lists)


def build_floor_out(task_list, onPreempt):
    waypoint_lists = {'RAMP_OUT': [[3,0],[3,-3],[0,-3],[0,0]],
                      'FLOOR_IN': [[3,0],[3,-3],[0,-3],[0,0]]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_FLOOR_OUT', waypoint_lists)


def build_floor_in(task_list, onPreempt):
    waypoint_lists = {'LINE': [[3,0],[3,-3],[0,-3],[0,0]],
                      'FLOOR_OUT': [[3,0],[3,-3],[0,-3],[0,0]]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_FLOOR_IN', waypoint_lists)


def build_line(task_list, onPreempt):
    waypoint_lists = {'FLOOR_OUT': [[3,0],[3,-3],[0,-3],[0,0]],
                      'LOAD_OFF_1': [[3,0],[3,-3],[0,-3],[0,0]],
                      'LOAD_OFF_2': [[3,0],[3,-3],[0,-3],[0,0]],
                      'LOAD_OFF_3': [[3,0],[3,-3],[0,-3],[0,0]],
                      'LOAD_ON_1': [[3,0],[3,-3],[0,-3],[0,0]],
                      'LOAD_ON_2': [[3,0],[3,-3],[0,-3],[0,0]],
                      'LOAD_ON_3': [[3,0],[3,-3],[0,-3],[0,0]]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LINE', waypoint_lists)


def build_station_1(task_list, onPreempt):
    waypoint_lists = { 'IN_BOX': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_STATION_1', waypoint_lists)


def build_station_2(task_list, onPreempt):
    waypoint_lists = { 'IN_BOX': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_STATION_2', waypoint_lists)


def build_station_3(task_list, onPreempt):
    waypoint_lists = { 'IN_BOX': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_STATION_3', waypoint_lists)


def build_load_off_1(task_list, onPreempt):
    waypoint_lists = { 'LINE': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_OFF_1', waypoint_lists)


def build_load_off_2(task_list, onPreempt):
    waypoint_lists = { 'LINE': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_OFF_2', waypoint_lists)


def build_load_off_3(task_list, onPreempt):
    waypoint_lists = { 'LINE': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_OFF_3', waypoint_lists)


def build_load_on_1(task_list, onPreempt):
    waypoint_lists = { 'LINE': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_ON_1', waypoint_lists)


def build_load_on_2(task_list, onPreempt):
    waypoint_lists = { 'LINE': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_ON_2', waypoint_lists)


def build_load_on_3(task_list, onPreempt):
    waypoint_lists = { 'LINE': [[3,0],[3,-3],[0,-3],[0,0]] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_ON_3', waypoint_lists)
