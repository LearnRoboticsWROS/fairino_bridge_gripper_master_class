from setuptools import find_packages, setup

package_name = 'fairino_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fra',
    maintainer_email='ros.master.ai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fairino_gripper_control = fairino_gripper.fairino_gripper_control:main',
            'gripper_action_server = fairino_gripper.gripper_action_server:main',
            'joint_state_merger = fairino_gripper.joint_state_merger:main'
        ],
    },
)
