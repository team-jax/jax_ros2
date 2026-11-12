from setuptools import setup

package_name = "vision_node"

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
    description="Camera node: /object_pose from a fixed test pose or from the camera program inference.json",
    license="TODO",
    entry_points={
        "console_scripts": [
            "vision_node = vision_node.vision_node:main",
        ],
    },
)
