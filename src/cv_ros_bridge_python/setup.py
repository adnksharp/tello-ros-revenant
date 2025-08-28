from setuptools import find_packages, setup

package_name = "cv_ros_bridge_python"

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
    maintainer="xtal",
    maintainer_email="manuel_lego1234@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "opencv_bridge = cv_ros_bridge_python.cv_ros_bridge:main",
            "opencv_bridge_talker = cv_ros_bridge_python.cv_ros_bridge_talker:main",
            "opencv_bridge_listener = cv_ros_bridge_python.cv_ros_bridge_listener:main",
            "image_publisher = cv_ros_bridge_python.image_publisher:main",
        ],
    },
)
