from setuptools import find_packages, setup
from glob import glob

package_name = 'navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Aggiunta dei file di launch, config e maps
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@todo.todo',
    description='Package for AMCL localization with custom map',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'waypoint_nav = navigation_pkg.waypoint_nav:main',
            	'point_transformer = navigation_pkg.point_transformer:main',
        ],
    },
)

