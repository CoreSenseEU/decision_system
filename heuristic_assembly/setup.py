from setuptools import find_packages, setup
from glob import glob

package_name = 'heuristic_assembly'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/behavior_trees/', glob('behavior_trees/*')),
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
            'assemble_decision_heuristic = heuristic_assembly.assemble_decision_heuristic_action_server:main',
            'adapt_decision_components = heuristic_assembly.adapt_decision_components_action_server:main',
        ],
    },
)
