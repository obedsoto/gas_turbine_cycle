from distutils.core import setup

import turbine_lib
import network_lib
import gases
import gas_dynamics
import functions

import py2exe

setup(

    name='turbine_lib',
    version='0.0.1',
    url='https://github.com/obedsoto/gas_turbine_cycle',
    platforms='any',
    options={'py2exe': {'bundle_files': 1, 'compressed': True}},
    zipfile=None,
    console=['turbine_lib.py']
)