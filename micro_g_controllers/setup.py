import os
from glob import glob

from setuptools import find_packages, setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "micro_g_controllers"

# Generate parameter libraries
generate_parameter_module(
    "grasp_selector_parameters",
    "params/grasp_selector_parameters.yaml",
)
generate_parameter_module(
    "linear_axis_controller_parameters",
    "params/linear_axis_controller_parameters.yaml",
)
generate_parameter_module(
    "pose_servoing_controller_parameters",
    "params/pose_servoing_controller_parameters.yaml",
)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        # Include all config files.
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Charles Dawson",
    maintainer_email="charles.dwsn@gmail.com",
    description="Controllers for micro-g robotics",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_servoing_controller=micro_g_controllers.pose_servoing_controller:main",
            "linear_axis_controller=micro_g_controllers.linear_axis_controller:main",
            "grasp_selector=micro_g_controllers.grasp_selector:main",
        ],
    },
)
