from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = []
d['scripts'] = ['custom_joint_state_publisher/custom_joint_state_publisher']
d['package_dir'] = {}

setup(**d)
