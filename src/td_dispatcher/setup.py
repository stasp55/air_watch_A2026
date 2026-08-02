from setuptools import find_packages, setup

package_name = "td_dispatcher"

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
    description="Агент-диспетчер: конечный автомат миссии, журнал решений.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dispatcher = td_dispatcher.dispatcher_node:main",
        ],
    },
)
