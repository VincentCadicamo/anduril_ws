from setuptools import find_packages, setup

package_name = "anduril_cv"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/gate_pose.launch.py"]),
    ],
    install_requires=["setuptools", "onnxruntime-gpu"],
    zip_safe=True,
    maintainer="Vincent Cadicamo",
    maintainer_email="vcadicamo@icloud.com",
    description="Single-stage YOLO-Pose gate detection and PnP pose estimation",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gate_pose_node = anduril_cv.gate_pose_node:main",
        ],
    },
)
