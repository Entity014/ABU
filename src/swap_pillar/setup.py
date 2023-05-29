from setuptools import setup

import os
from glob import glob

package_name = 'abu_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share',package_name), glob("launch/*launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poom',
    maintainer_email='poom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		"drive_node = abu_package.ps4:main",
		"shoot_node = abu_package:main",
		"feed_node  = abu_package:main",
		"color_node = abu_package.detectColor:main",
		"joyCon_node = abu_package.joyConnection:main"
        ],
    },
)
