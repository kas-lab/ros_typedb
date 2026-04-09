from setuptools import find_packages
from setuptools import setup


package_name = 'ros_typedb_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pydot'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Tools for TypeDB 3 workflows, including schema and function diagram generation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'typedb_schema_diagram = ros_typedb_tools.typedb_schema_diagram:main',
            'typedb_rule_diagram = ros_typedb_tools.typedb_rule_diagram:main',
        ],
    },
)
