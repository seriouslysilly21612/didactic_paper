from setuptools import setup

package_name = 'openvino_detector_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='YOLOv5 object detector with OpenVINO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openvino_detector_node = openvino_detector_pkg.openvino_detector_node:main',
        ],
    },
)
