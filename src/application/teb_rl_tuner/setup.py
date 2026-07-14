#!/usr/bin/env python3

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup


setup(
    **generate_distutils_setup(
        packages=["teb_rl_tuner"],
        package_dir={"": "src"},
    )
)
