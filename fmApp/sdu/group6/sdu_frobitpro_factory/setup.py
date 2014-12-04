#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts = ['mission_planners/follow_square.py', 'mission_planners/topic_controlled_mission.py', 'scripts/gui_node.py'],
  packages = ['rsd_smach'],
  package_dir = {'': 'src'}
)

setup(**d)
