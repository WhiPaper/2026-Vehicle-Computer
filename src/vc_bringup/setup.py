from glob import glob
import os

from setuptools import find_packages, setup


package_name = "vc_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vehicle Computer Maintainers",
    maintainer_email="me@whipaper.net",
    description="Launch and configuration for the RPi5 vehicle computer.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "diagnostics_mux = vc_bringup.diagnostics:diagnostics_mux_main",
            "fake_ecu = vc_bringup.fake_ecu:main",
            "state_estimation_monitor = vc_bringup.diagnostics:state_estimation_monitor_main",
        ],
    },
)
