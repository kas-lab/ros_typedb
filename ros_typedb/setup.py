import os
from glob import glob
from setuptools import setup

package_name = 'ros_typedb'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='ROS interface for TypeDB',
    license='Apache License 2.0',
    extras_require = {
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_typedb = ros_typedb.ros_typedb_node:main'
        ],
    },
)
