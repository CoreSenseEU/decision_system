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
            'update_alternatives = abstract_decision_components.update_alternatives_prolog_action_server:main',
            'update_cues = abstract_decision_components.update_cues_prolog_action_server:main',

            'cue_prolog = abstract_decision_components.cue.cue_prolog_node:main',

            'assess_action_server = abstract_decision_components.assess.assess_action_server:main',

            'aggregate_preferences_action_server = abstract_decision_components.aggregate.aggregate_preferences_action_server:main',
            'aggregate_utility_boolean_action_server = abstract_decision_components.aggregate.aggregate_utility_boolean_action_server:main',
            'aggregate_utility_signed_action_server = abstract_decision_components.aggregate.aggregate_utility_signed_action_server:main',
            'aggregate_utility_sum_action_server = abstract_decision_components.aggregate.aggregate_utility_sum_action_server:main',
            'aggregate_multi_value_utility_action_server = abstract_decision_components.aggregate.aggregate_multi_value_utility_action_server:main',

            'order_condorcet_extension_action_server = abstract_decision_components.order.order_condorcet_extension_action_server:main',
            'order_lexicographical_action_server = abstract_decision_components.order.order_lexicographical_action_server:main',
            'order_dominating_action_server = abstract_decision_components.order.order_dominating_action_server:main',

            'take_best_action_server = abstract_decision_components.take.take_best_action_server:main',
            'eliminate_worst_action_server = abstract_decision_components.take.eliminate_worst_action_server:main',

            'accept_always_action_server = abstract_decision_components.accept.accept_always_action_server:main',
            'accept_size_action_server = abstract_decision_components.accept.accept_size_action_server:main',
            'accept_satisficing_action_server = abstract_decision_components.accept.accept_satisficing_action_server:main',
            'accept_dominating_action_server = abstract_decision_components.accept.accept_dominating_action_server:main',
        ],
    },
)
