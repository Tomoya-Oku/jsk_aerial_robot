from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# A regular package resolves consistently in both catkin devel and install spaces.
setup(**generate_distutils_setup(packages=['dracomancer_mkii_ros'], package_dir={'': 'src'}))
