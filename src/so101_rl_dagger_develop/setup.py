import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_rl_dagger_develop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rl_dagger_develop.launch.py"]),
        ("share/" + package_name + "/config", glob("config/*")),
        # setuptools>=80 puts console_scripts in bin/, ros2 run needs lib/<pkg>/ wrapper
        (os.path.join("lib", package_name), glob("wrapper/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maxtensor",
    maintainer_email="maxtensor@163.com",
    description="DAgger student 实机部署开发包（纯视觉 student + 实机相机 + ros2_control）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rl_dagger_develop_node = so101_rl_dagger_develop.dagger_develop_node:main",
            "rl_dagger_camera_node = so101_rl_dagger_develop.dagger_camera_node:main",
        ],
    },
)
