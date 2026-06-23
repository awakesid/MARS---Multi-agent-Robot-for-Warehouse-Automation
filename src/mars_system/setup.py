from setuptools import find_packages, setup

package_name = 'mars_system'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node    = mars_system.camera_node:main',
        'aruco_detector = mars_system.aruco_detector:main',
        'test_drive = mars_system.test_drive:main',
        'task_manager = mars_system.task_manager:main',
         'task_manager_single     = mars_system.task_manager_single:main',
    'task_manager_priority   = mars_system.task_manager_priority:main',
    'task_manager_thread   = mars_system.task_manager_thread:main',
        ],
    },
)
