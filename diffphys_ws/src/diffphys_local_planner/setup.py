from setuptools import setup

package_name = 'diffphys_local_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/single_run_diffphys.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='DiffPhysDrone local planner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'local_planner_node = diffphys_local_planner.local_planner_node:main',
            'validator = diffphys_local_planner.validator:main',
        ],
    },
)
