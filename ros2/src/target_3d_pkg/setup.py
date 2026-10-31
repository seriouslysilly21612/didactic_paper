from setuptools import find_packages, setup

package_name = 'target_3d_pkg'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='The 3-D target in the camera and in the robot base frame',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_target_3d_node = target_3d_pkg.pick_target_3d_node:main',
            'pick_target_base_node = target_3d_pkg.pick_target_base_node:main',
        ],
    },
)
