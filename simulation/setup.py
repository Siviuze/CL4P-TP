#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages


setup(
    name='claptrap_simu',
    version='0.1.0',
    description='Simulation and log processing for Claptrap',
    long_description=open('README.md').read(),
    packages=find_packages('src'),
    package_dir={'':'src'},
    entry_points = {
        'console_scripts': ['claptrap_plotter=claptrap_simu.log_handling.plotter:main',
                            'claptrap_replay=claptrap_simu.log_handling.viewer3D:main'],
    },
    package_data={'claptrap_simu': ['data/*.urdf','data/*.dae']},
    zip_safe=False)
