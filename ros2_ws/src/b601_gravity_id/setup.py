from setuptools import find_packages, setup

package_name = "b601_gravity_id"

setup(
    name=package_name,
    version="1.1.3",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/gravity_params.yaml"]) if __import__("os").path.exists("config/gravity_params.yaml") else ("share/" + package_name, []),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dave",
    maintainer_email="dave@example.com",
    description="B601 重力/摩擦辨识与 MIT 力控",
    license="MIT",
)
