__author__ = 'kristjan'

from rsd_smach.navigatestates import navigate_base_state
import csv
import os
import rospy

wpts = {}
with open(os.path.dirname(os.path.realpath(__file__))+'/waypoints.csv', 'rb') as wptfile:
    reader = csv.reader(wptfile, delimiter=',', quotechar='|')
    for row in reader:
        wpts[row[0]] = [float(row[1]),float(row[2])]
print wpts

def build_dispenser(task_list, onPreempt):
    waypoint_lists = {'IN_BOX': [wpts.get('inbox_station3', [0,0]), wpts.get('dispenser_nexttoend',[0,0]), wpts.get('dispenser_atend',[0,0])]}
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_DISPENSER', waypoint_lists)


def build_in_box(task_list, onPreempt):
    waypoint_lists = {'RAMP_IN': [wpts.get('rampin_exit', [0,0]), wpts.get('inbox_entrance', [0,0])],
                      'STATION_1': [wpts.get('inbox_station1', [0,0])],
                      'STATION_2': [wpts.get('inbox_station1', [0,0])],
                      'STATION_3': [wpts.get('inbox_station1', [0,0])]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_IN_BOX', waypoint_lists)


def build_ramp_out(task_list, onPreempt):
    waypoint_lists = {'DISPENSER': [wpts.get('rampout_entrance', [0,0])],
                      'IN_BOX': [wpts.get('inbox_entrance', [0,0]), wpts.get('rampout_entrance', [0,0])]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_RAMP_OUT', waypoint_lists)


def build_ramp_in(task_list, onPreempt):
    waypoint_lists = {'FLOOR_IN': [wpts.get('floorin_almostinfrontoframp', [0,0]), wpts.get('floorin_infrontoframp', [0,0]), wpts.get('rampin_entrance', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_RAMP_IN', waypoint_lists)


def build_floor_out(task_list, onPreempt):
    waypoint_lists = {'RAMP_OUT': [wpts.get('rampout_exit', [0,0]), wpts.get('floorout_entrance', [0,0])],
                      'FLOOR_IN': [wpts.get('floorin_tofloorout', [0,0]), wpts.get('floorout_tofloorin', [0,0])]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_FLOOR_OUT', waypoint_lists)


def build_floor_in(task_list, onPreempt):
    waypoint_lists = {'LINE': [wpts.get('line_lowerleftcorner', [0,0]), wpts.get('line_upperleftcorner', [0,0]), wpts.get('line_upperrightcorner', [0,0]), wpts.get('line_lowerrightcorner', [0,0]), wpts.get('floorin_entrance', [0,0])],
                      'FLOOR_OUT': [wpts.get('floorout_tofloorin', [0,0]), wpts.get('floorin_tofloorout', [0,0])]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_FLOOR_IN', waypoint_lists)


def build_line(task_list, onPreempt):
    waypoint_lists = {'FLOOR_OUT': [wpts.get('floorout_almostinfrontofline', [0,0]), wpts.get('floorout_infrontofline', [0,0]), wpts.get('line_lowerleftcorner', [0,0])],
                      'LOAD_OFF_1': [wpts.get('line_cell1conveyor', [0,0])],
                      'LOAD_OFF_2': [wpts.get('line_cell2conveyor', [0,0])],
                      'LOAD_OFF_3': [wpts.get('line_cell3conveyor', [0,0])],
                      'LOAD_ON_1': [wpts.get('line_cell1exit', [0,0])],
                      'LOAD_ON_2': [wpts.get('line_cell2exit', [0,0])],
                      'LOAD_ON_3': [wpts.get('line_cell3exit', [0,0])]
                      }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LINE', waypoint_lists)


def build_station_1(task_list, onPreempt):
    waypoint_lists = { 'IN_BOX': [wpts.get('inbox_station1', [0,0]), wpts.get('station1', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_STATION_1', waypoint_lists)


def build_station_2(task_list, onPreempt):
    waypoint_lists = { 'IN_BOX': [wpts.get('inbox_station2', [0,0]), wpts.get('station2', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_STATION_2', waypoint_lists)


def build_station_3(task_list, onPreempt):
    waypoint_lists = { 'IN_BOX': [wpts.get('inbox_station3', [0,0]), wpts.get('station3', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_STATION_3', waypoint_lists)


def build_load_off_1(task_list, onPreempt):
    waypoint_lists = { 'LINE': [wpts.get('cell1_conveyor', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_OFF_1', waypoint_lists, reverse_mode=True)


def build_load_off_2(task_list, onPreempt):
    waypoint_lists = { 'LINE': [wpts.get('cell2_conveyor', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_OFF_2', waypoint_lists, reverse_mode=True)


def build_load_off_3(task_list, onPreempt):
    waypoint_lists = { 'LINE': [wpts.get('cell3_conveyor', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_OFF_3', waypoint_lists, reverse_mode=True)


def build_load_on_1(task_list, onPreempt):
    waypoint_lists = { 'LINE': [wpts.get('cell1_exit', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_ON_1', waypoint_lists, reverse_mode=True)


def build_load_on_2(task_list, onPreempt):
    waypoint_lists = { 'LINE': [wpts.get('cell2_exit', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_ON_2', waypoint_lists, reverse_mode=True)


def build_load_on_3(task_list, onPreempt):
    waypoint_lists = { 'LINE': [wpts.get('cell3_exit', [0,0])] }
    return navigate_base_state.build(task_list, onPreempt, 'NAVIGATE_LOAD_ON_3', waypoint_lists, reverse_mode=True)
