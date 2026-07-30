from glob import glob
from setuptools import setup

package_name = "piper_calibration"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dream",
    maintainer_email="dream@example.com",
    description="Real-robot eye-to-hand calibration for Piper",
    license="MIT",
    entry_points={
        "console_scripts": [
            "calibration_node = piper_calibration.calibration_node:main",
            "verify_calibration_moveit = piper_calibration.verify_calibration_moveit:main",
        ],
    },
)
