"""Setup for the ros_typedb_examples package."""

from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'ros_typedb_examples'
typedb_test_data_path = os.path.join(
    '..',
    'ros_typedb',
    'test',
    'typedb_test_data',
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'data'),
            glob('data/*.tql')),
        (os.path.join('share', package_name, 'data', 'typedb_test_data'),
            [
                os.path.join(typedb_test_data_path, 'schema.tql'),
                os.path.join(typedb_test_data_path, 'data.tql'),
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Example launch files and TypeDB models for ros_typedb.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
