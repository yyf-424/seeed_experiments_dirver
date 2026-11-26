from setuptools import find_packages, setup

package_name = "b601_safety"

setup(
    name=package_name,
    version="1.1.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 生成的参数文件（scripts/sync_config.py 从 b601_project.yaml 产出）
        ("share/" + package_name + "/config", ["config/safety_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dave",
    maintainer_email="dave@example.com",
    description="B601 独立安全监控节点",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_monitor = b601_safety.safety_monitor:main",
        ],
    },
)
