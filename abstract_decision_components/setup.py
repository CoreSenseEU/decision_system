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
            'cue_prolog = abstract_decision_components.cue.cue_prolog_node:main',

            'assess = abstract_decision_components.assess.assess_node:main',
            'assess_action_server = abstract_decision_components.assess.assess_action_server:main',

            'aggregate_preferences = abstract_decision_components.aggregate.aggregate_preferences_node:main',
            'aggregate_utility_boolean = abstract_decision_components.aggregate.aggregate_utility_boolean_node:main',
            'aggregate_utility_signed = abstract_decision_components.aggregate.aggregate_utility_signed_node:main',
            'aggregate_utility_sum = abstract_decision_components.aggregate.aggregate_utility_sum_node:main',
            'aggregate_multi_value_utility = abstract_decision_components.aggregate.aggregate_multi_value_utility_node:main',

            'order_condorcet_extension = abstract_decision_components.order.order_condorcet_extension_node:main',
            'order_lexicographical = abstract_decision_components.order.order_lexicographical_node:main',
            'order_dominating = abstract_decision_components.order.order_dominating_node:main',

            'take_best = abstract_decision_components.take.take_best_node:main',
            'eliminate_worst = abstract_decision_components.take.eliminate_worst_node:main',

            'accept_always = abstract_decision_components.accept.accept_always_node:main',
            'accept_size = abstract_decision_components.accept.accept_size_node:main',
            'accept_satisficing = abstract_decision_components.accept.accept_satisficing_node:main',
            'accept_dominating = abstract_decision_components.accept.accept_dominating_node:main',
        ],
    },
)
