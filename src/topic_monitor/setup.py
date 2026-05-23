from setuptools import setup

package_name = "topic_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "rate_monitor = topic_monitor.topic_rate_monitor:main",
            "topic_tester = topic_monitor.topic_tester:main",
        ],
    },
)
