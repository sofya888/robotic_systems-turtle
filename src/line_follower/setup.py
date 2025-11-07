from setuptools import setup

package_name = 'line_follower'

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/line_follower"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/line_follower.launch.py"]),
        ("share/" + package_name + "/maps", ["maps/line_map.yaml", "maps/line_map.pgm"]),
    ],
    install_requires=["setuptools", "numpy", "Pillow", "PyYAML"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Line follower (ROS 2, turtlesim demo)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "line_follower = line_follower.line_follower:main",
        ],
    },
)
