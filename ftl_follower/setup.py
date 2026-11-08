from setuptools import setup

package_name = "ftl_follower"

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
    maintainer="Gustavo",
    maintainer_email="gujoseleite@gmail.com",
    description="Follow the leader: pose estimator and path follower.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_estimator = ftl_follower.pose_estimator:main",
            "follower = ftl_follower.follower:main",
        ],
    },
)
