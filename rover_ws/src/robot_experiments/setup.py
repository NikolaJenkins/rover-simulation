from setuptools import find_packages, setup

package_name = "robot_experiments"

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
    maintainer="njoy",
    maintainer_email="naojoy5@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "detector = robot_experiments.scan_monitor:main",
            "odometry = robot_experiments.odom_drive:main",
            "navigation = robot_experiments.waypoint_nav:main",
        ],
    },
)
