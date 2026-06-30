from setuptools import find_packages, setup

package_name = "anduril_ekf"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/ekf.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vincent Cadicamo",
    maintainer_email="vcadicamo@icloud.com",
    description="Error-state EKF fusing IMU and PnP gate detections",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drone_ekf_node = anduril_ekf.drone_ekf_node:main",
        ],
    },
)
