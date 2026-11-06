import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_camera_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # setuptools>=80 puts console_scripts in bin/, ros2 run needs lib/<pkg>/ wrapper
        (os.path.join("lib", package_name), glob("wrapper/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maxtensor",
    maintainer_email="maxtensor@163.com",
    description="SO-101 相机内参/手眼标定（迁移自 so101-ros-physical-ai, Apache-2.0）",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_intrinsic_calibration_node = so101_camera_calibration.camera_intrinsic_calibration_node:main",
            "handeye_node = so101_camera_calibration.handeye_node:main",
            "compare_views_node = so101_camera_calibration.compare_views_node:main",
        ],
    },
)
