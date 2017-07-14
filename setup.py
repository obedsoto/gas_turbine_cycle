from distutils.core import setup

import turbine_lib
import network_lib
import gases
import gas_dynamics
import functions

import py2exe

setup(
    options={'py2exe': {'bundle_files': 1, 'compressed': True}},
    zipfile=None,
    console=['turbine_lib.py']
)