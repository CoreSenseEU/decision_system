from setuptools import find_packages, setup

package_name = 'heuristic_assembly'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/behavior_trees/', ['behavior_trees']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tyler Olson',
    maintainer_email='t.k.olson@student.tudelft.nl',
    description='Assemble and modify decision-making heuristics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
