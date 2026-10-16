from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['alicia_grasp_modes'], package_dir={'': 'src'}))
