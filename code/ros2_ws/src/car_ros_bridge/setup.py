from setuptools import find_packages, setup

setup(name="car_ros_bridge", version="0.1.0", packages=find_packages(), install_requires=["setuptools"], zip_safe=True, data_files=[("share/ament_index/resource_index/packages", ["resource/car_ros_bridge"]), ("share/car_ros_bridge", ["package.xml"])], entry_points={"console_scripts": ["d500_localization_node = car_ros_bridge.d500_localization_node:main"]})
