from setuptools import find_packages, setup
from glob import glob

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Aggiunta dei file di launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'detector = perception_pkg.detector:main',
        	'acquisition = perception_pkg.acquisition:main',
        	'visualization = perception_pkg.visualization:main',
            'manager = perception_pkg.manager:main',
            'compute_waypoint = perception_pkg.compute_waypoint:main',
        ],
    },
)
