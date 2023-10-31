from setuptools import find_packages, setup
import os
from glob import glob

package_name = "micro_g_vision"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Charles Dawson",
    maintainer_email="charles.dwsn@gmail.com",
    description="Vision system for micro-g robotics",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_tracker=micro_g_vision.object_tracker:main"
        ],
    },
)
