#!/usr/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d1 = generate_distutils_setup(
	packages=['master_controller'],
	package_dir={'': 'src'}
)

setup(**d1)
