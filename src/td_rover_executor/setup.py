from setuptools import find_packages, setup

package_name = "td_rover_executor"

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
    description="Исполнитель ровера: навык goto_cell поверх Nav2 и маска запретных клеток.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goto_cell = td_rover_executor.goto_cell_server:main",
            "keepout_publisher = td_rover_executor.keepout_publisher:main",
            "rover_status = td_rover_executor.status_node:main",
        ],
    },
)
