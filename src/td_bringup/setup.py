from glob import glob

from setuptools import setup

package_name = "td_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="town-dozor team",
    maintainer_email="team@town-dozor.local",
    description="Запуск системы «Дозор»: конфигурация поля, launch-профили, инструменты.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibrate_field = td_bringup.calibrate_field:main",
            "make_templates = td_bringup.make_templates:main",
            "ghost_enemy = td_bringup.ghost_enemy:main",
        ],
    },
)
