from glob import glob

from setuptools import find_packages, setup

package_name = "so101_motion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz") + glob("config/*.csv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miguel-das",
    maintainer_email="47686612+miguel-das@users.noreply.github.com",
    description="MoveItPy motion nodes for the SO101 arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motion_server = so101_motion.motion_server:main",
            "send_waypoints = so101_motion.send_waypoints:main",
        ],
    },
)
