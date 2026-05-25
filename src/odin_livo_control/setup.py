from setuptools import setup


package_name = "odin_livo_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="local",
    maintainer_email="dev@example.com",
    description="Desktop control panel for Odin driver and FAST-LIVO2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "odin_livo_gui = odin_livo_control.gui:main",
        ],
    },
)
