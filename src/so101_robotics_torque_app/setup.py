import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_robotics_torque_app"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maxtensor",
    maintainer_email="maxtensor@todo.todo",
    description="SO-ARM101 torque-control learning app: draw_cube acceptance demo",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "torque_control_node = so101_robotics_torque_app.torque_control_node:main",
            "draw_cube_node = so101_robotics_torque_app.draw_cube_node:main",
        ],
    },
)
