from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['patrol_robot'],
    package_dir={'': 'utilities'}
)

setup(**setup_args)
