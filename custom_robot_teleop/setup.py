from setuptools import find_packages, setup

package_name = "custom_robot_teleop"

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
    maintainer="bulingen",
    maintainer_email="adamlecorney@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_to_single_array = custom_robot_teleop.cmd_vel_to_single_array:main",
        ],
    },
)
