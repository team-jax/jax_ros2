from setuptools import setup

package_name = "arm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "calibration.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeajune0221",
    maintainer_email="jeajune11117@gmail.com",
    description="Sim joint angles to the real JAX arm",
    license="TODO",
    entry_points={
        "console_scripts": [
            "arm_control_node = arm_control.arm_control_node:main",
            "calibrate = arm_control.calibrate:main",
            "play_trajectory = arm_control.play:main",
            "pick_place = arm_control.pick_place:main",
        ],
    },
)
