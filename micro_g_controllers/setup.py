from setuptools import find_packages, setup

package_name = "micro_g_controllers"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "pose_servoing_controller=micro_g_controllers.pose_servoing_controller:main"
        ],
    },
)
