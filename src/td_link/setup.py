from setuptools import find_packages, setup

package_name = "td_link"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="town-dozor team",
    maintainer_email="team@town-dozor.local",
    description="Мост между агентами: протокол поверх MQTT и прокси ROS-действий.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agent_link = td_link.agent_link_node:main",
            "dispatcher_link = td_link.dispatcher_link_node:main",
        ],
    },
)
