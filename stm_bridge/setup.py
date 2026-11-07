from setuptools import setup

package_name = "stm_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeajune0221",
    maintainer_email="jeajune11117@gmail.com",
    description="/servo/command to CAN_STM servo frames",
    license="TODO",
    entry_points={
        "console_scripts": [
            "stm_bridge_node = stm_bridge.stm_bridge_node:main",
        ],
    },
)
