from setuptools import find_packages, setup

package_name = 'prolog_kb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tyler Olson',
    maintainer_email='t.k.olson@student.tudelft.nl',
    description='An example knowledge base implemented in Prolog',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prolog_server = prolog_kb.prolog_server_node:main',
            'prolog_pose_factory = prolog_kb.prolog_pose_factory_node:main',
            'get_drop_locations_prolog_adapter = prolog_kb.get_drop_locations_prolog_adapter_node:main',
            # 'get_object_info_prolog_adapter = prolog_kb.get_object_info_prolog_adapter_node:main',
            'get_objects_in_room_prolog_adapter = prolog_kb.get_objects_in_room_prolog_adapter_node:main',
            'pick_object_prolog_adapter = prolog_kb.pick_object_prolog_adapter_node:main',
            'place_object_prolog_adapter = prolog_kb.place_object_prolog_adapter_node:main',
        ],
    },
)
