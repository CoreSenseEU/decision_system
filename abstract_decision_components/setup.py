from setuptools import find_packages, setup

package_name = 'abstract_decision_components'

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
    description='Abstract decision making components',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'accept_always = abstract_decision_components.accept_always_node:main',
            'accept_size = abstract_decision_components.accept_size_node:main',
            'accept_satisficing = abstract_decision_components.accept_satisficing_node:main',
            'accept_domminating = abstract_decision_components.accept_dominating_node:main',
            'take_best = abstract_decision_components.take_best_node:main',
            'eliminate_worst = abstract_decision_components.eliminate_worst_node:main',
            'order_copeland = abstract_decision_components.order_copeland_node:main',
            'order_lexicographical = abstract_decision_components.order_lexicographical_node:main',
            'order_pareto_fronts = abstract_decision_components.order_pareto_fronts_node:main',
            'order_sequential_majority_comparison = abstract_decision_components.order_sequential_majority_comparison_node:main',
        ],
    },
)
