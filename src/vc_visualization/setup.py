from glob import glob
import os

from setuptools import find_packages, setup


package_name = "vc_visualization"

setup(
    name=package_name,
    version="0.2.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vehicle Computer Maintainers",
    maintainer_email="me@whipaper.net",
    description="PC-side visualization, replay, and Gazebo simulation tools.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulation_health = vc_visualization.simulation_health:main",
        ],
    },
)
