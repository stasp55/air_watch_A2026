from setuptools import find_packages, setup

package_name = "td_world"

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
    description="Модель мира: сетка поля, привязка координат, слияние детекций.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "world_state = td_world.world_state_node:main",
            "field_tf = td_world.field_tf_node:main",
        ],
    },
)
