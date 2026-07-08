from setuptools import find_packages, setup

package_name = "nexus_sand_mpc"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/sand_mpc.yaml"]),
        (f"share/{package_name}/launch", ["launch/sand_mpc.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "do-mpc", "casadi"],
    zip_safe=True,
    maintainer="Charles",
    maintainer_email="charles@nexus.org",
    description="ROS 2 migration of the sand-slip MPC command compensator.",
    license="All Rights Reserved",
    entry_points={
        "console_scripts": [
            "sand_mpc_compensator = nexus_sand_mpc.sand_mpc_node:main",
        ],
    },
)
