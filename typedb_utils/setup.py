from setuptools import find_packages
from setuptools import setup


package_name = 'typedb_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'typedb-driver'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Reusable TypeDB 3 Python utilities and interface layer',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
