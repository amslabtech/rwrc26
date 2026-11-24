# catkin_python_setup() から呼ばれる。直接 python setup.py を実行しないこと
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=["road_edge_detector"],
    package_dir={"": "src"},
))
