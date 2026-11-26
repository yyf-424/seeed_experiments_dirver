from setuptools import find_packages, setup

package_name = "b601_demos"

setup(
    name=package_name,
    version="1.1.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/target_web.launch.py"]),
        ("share/" + package_name + "/web", [
            "web/index.html", "web/style.css", "web/app.js", "web/robot-view.js", "web/tcp-ik.js",
        ]),
        ("share/" + package_name + "/web/vendor", [
            "web/vendor/three.module.min.js", "web/vendor/OrbitControls.js",
            "web/vendor/STLLoader.js", "web/vendor/THREE-LICENSE.txt",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dave",
    maintainer_email="dave@example.com",
    description="B601 教学演示节点",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 命令名 = 模块路径:入口函数
            # 左边是 ros2 run 用的可执行名，右边是包里真正的节点实现
            "joint_state_monitor = b601_demos.joint_state_monitor:main",
            "move_to_joints = b601_demos.move_to_joints:main",
            "demo_trajectory = b601_demos.demo_trajectory:main",
            "gripper = b601_demos.gripper_control:main",
            "moveit_pose_goal = b601_demos.moveit_pose_goal:main",
            "target_web = b601_demos.target_web:main",
            "target_arbiter = b601_demos.target_arbiter:main",
            "target_bridge = b601_demos.target_bridge:main",
            "mit_recenter = b601_demos.mit_recenter:main",
            "mit_recenter_call = b601_demos.mit_recenter_client:main",
        ],
    },
)
