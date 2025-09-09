from setuptools import find_packages, setup

package_name = "osm2qr"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/qr2geo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vlkjan6",
    maintainer_email="vlkjan6@fel.cvut.cz",
    description="TODO: Package description",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "qr2geo = osm2qr.qr2geo:main",
        ],
    },
)
