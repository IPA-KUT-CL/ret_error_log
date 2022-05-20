#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ret_error_logger'],
    scripts=['scripts/ros_segfault'],
    package_dir={'': 'src'}
)

setup(**d)