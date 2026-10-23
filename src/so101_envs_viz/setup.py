import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_envs_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        # setuptools>=80 puts console_scripts in bin/, ros2 run needs lib/<pkg>/ wrapper
(os.path.join("lib", package_name), ["wrapper/viz_node"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maxtensor",
    maintainer_email="maxtensor@todo.todo",
    description="SO101 gymnasium envs RViz visualization",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "viz_node = so101_envs_viz.viz_node:main",
        ],
    },
)
