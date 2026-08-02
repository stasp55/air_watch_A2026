from setuptools import find_packages, setup

package_name = "td_perception_rover"

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
    description="Зрение ровера: чтение финишной клетки плагинами (шаблоны, VLM, OCR).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "label_reader = td_perception_rover.label_reader_node:main",
        ],
    },
)
