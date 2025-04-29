# Copyright 2025 KAS-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from decision_msgs.msg import Preference
from decision_msgs.srv import AssessAlternatives, PrologQuery


class CuePrologNode(Node):
    """
    A service to query a prolog knowledge base to answer questions about alternatives.
    This assumes answers about alternatives are independent.

    :param query: A prolog query to submit to the database where the variable `_A`
        is used for input alternatives and `V` is used for the desired value.
    :param knowledge_base: A path to the prolog knowledge base.
    """
    def __init__(self):
        super().__init__('cue_prolog_service')
        self.get_logger().info('Starting CUE node which queries a prolog knowledge base')

        self.declare_parameter('query', Parameter.Type.STRING) # prolog query referring to the alternative as A

        self.srv_ = self.create_service(
                AssessAlternatives,
                'assess',
                self.query_prolog_cb,
                # TODO: can this be ReentrantCallbackGroup? pyswip might complain...
                callback_group = MutuallyExclusiveCallbackGroup())

        self.prolog_client_ = self.create_client(
                PrologQuery,
                'query',
                callback_group = MutuallyExclusiveCallbackGroup())


    def query_prolog_cb(self, request, result):
        if len(request.alternatives) < 1:
            self.get_logger().error('No alternatives to assess')
            return result

        query = self.get_parameter('query').value
        self.get_logger().info(f'Assessing alternatives {request.alternatives} with policy: cue_prolog "{query}"')
        try:
            preferences = [Preference(alternative=a, score=self.assess_one(a)) for a in request.alternatives]
        except ValueError as e:
            self.get_logger().error(str(e))
            return result

        result.preferences = preferences
        return result

    def assess_one(self, alternative):
        """
        :raises ValueError: If a query could not be answered or answer was
            missing a binding for the score variable `V`.
        """
        query = self.get_parameter('query').value
        request = PrologQuery.Request()

        # TODO: make this more robust
        request.query.clause = query.replace('_A', f"'{alternative.id}'")
        request.maxresult = 1

        response = self.call_service(self.prolog_client_, request)
        if not response.success or len(response.answers) == 0:
            raise ValueError(f'Failed to answer query: "{request.query.clause}"')

        for answer in response.answers:
            for binding in answer.bindings:
                if binding.key == "V":
                    return float(binding.value)

        raise ValueError(f'No bindings for "V" in answers to query: "{request.query.clause}"')

    # def assess_all(self, alternatives):
    #     query = self.get_parameter('query').value
    #     # TODO: make this more robust
    #     query.replace('_A', 'A')
    #
    #     request = PrologQuery.Request()
    #     request.query.clause = query
    #     request.maxresult = -1 # Get all results
    #
    #     response = self.call_service(self.prolog_client_, request)
    #     if not response.success or len(response.answers) == 0:
    #         self.get_logger().error(f'Failed to answer query: "{query}"')
    #         return None
    #
    #     preferences = []
    #     alternatives = []
    #     scores = []
    #     for answer in response.answers:
    #         for binding in answer.bindings:
    #             if binding.key == "V":
    #                 scores.append(float(binding.value))
    #             elif binding.key == "A":
    #                 alternative = str(binding.value)
    #                 if alternative not in alternatives:
    #                     alternatives.append()
    #                 else:
    #                     # TODO: would we ever expect/want to handle multiple possible scores?
    #                     self.get_logger().error(f'Multiple bindings for alternative "{alternative}" for query: "{query}"')
    #                     return None
    #         if len(alternatives) != len(scores): # this should only be possible if the ROS message passing was wrong
    #             self.get_logger().error(f'Missing a binding in answer to query: "{query}"')
    #
    #     for a, s in zip(alternatives, scores):
    #         preferences.append(Preference(alternative=a, score=s))
    #
    #     return preferences

    def call_service(self, cli, request):
        """
        Adopted from https://github.com/kas-lab/krr_mirte_skills/blob/main/krr_mirte_skills/krr_mirte_skills/get_object_info.py
        """
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    node = CuePrologNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
