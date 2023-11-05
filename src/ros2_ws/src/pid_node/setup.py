from setuptools import setup

package_name = 'pid_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bigman',
    maintainer_email='ha4863519@gmail.com',
    description='PID control system for sub motors',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = pid_node.PID_Node:main',
                'listener = pid_node.PID_Controller:main',
        ],
    },
)
