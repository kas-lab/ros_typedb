"""Package setup for ros_typedb_benchmark."""

from setuptools import find_packages, setup

package_name = 'ros_typedb_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['profiles/*.json'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Stress-testing and fault-injection benchmark tools for ros_typedb.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'ros_typedb_stress_experiment = '
                'ros_typedb_benchmark.ros_typedb_stress_experiment:main'
            ),
            (
                'ros_typedb_fake_query_service = '
                'ros_typedb_benchmark.fake_query_service:fake_query_service_main'
            ),
        ],
    },
)
