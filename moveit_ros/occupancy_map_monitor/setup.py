## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages, setup

packages = find_packages('src')
packages.remove("moveit")
print(">>>>")
print(packages)

d = generate_distutils_setup(
    packages=packages,
    package_dir={'': 'src'}
)

setup(**d)
