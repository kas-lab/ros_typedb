from pathlib import Path

from setuptools import find_packages
from setuptools import setup


package_name = 'typedb_utils'
root = Path(__file__).parent
readme_path = root / 'README.md'
if readme_path.exists():
    readme = readme_path.read_text(encoding='utf-8')
else:
    # ament_python may invoke a copied setup.py from the build tree, where the
    # project README is not present alongside the generated script.
    readme = 'Reusable TypeDB 3 Python utilities and interface layer'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'typedb-driver'],
    python_requires='>=3.10',
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Reusable TypeDB 3 Python utilities and interface layer',
    long_description=readme,
    long_description_content_type='text/markdown',
    license='Apache-2.0',
    url='https://github.com/Rezenders/ros_typedb',
    project_urls={
        'Documentation': 'https://rezenders.github.io/ros_typedb/',
        'Repository': 'https://github.com/Rezenders/ros_typedb',
        'Issues': 'https://github.com/Rezenders/ros_typedb/issues',
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Database',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
