from setuptools import find_packages, setup

package_name = "micro_g_state_machine"

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
    maintainer="Evan Palmer",
    maintainer_email="evanp922@gmail.com",
    description="State machine for micro-g",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["state_machine = micro_g_state_machine.state_machine:main"],
    },
)
