#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['python_op3'],
    package_dir={'': 'src'},
    scripts=['scripts/op3_test']
)

setup(**d)
