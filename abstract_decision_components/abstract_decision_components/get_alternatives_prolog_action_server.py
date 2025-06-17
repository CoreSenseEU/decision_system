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
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from decision_msgs.msg import AlternativeArray, Alternative
from decision_msgs.action import GetAlternatives
from decision_msgs.srv import PrologQuery


class GetAlternativesPrologActionServer(Node):
    """
    An action server to query a prolog knowledge base to get alternatives.
    """
    def __init__(self):
        super().__init__('get_alternatives_prolog_action_server')
        self.get_logger().info('Starting GET_ALTERNATIVES action server which queries a prolog knowledge base')

        self.action_server_ = ActionServer(
                self,
                GetAlternatives,
                'GetAlternatives',
                self.get_alternatives_cb)

        self.prolog_client_ = self.create_client(
                PrologQuery,
                'query',
                callback_group = MutuallyExclusiveCallbackGroup())

        self.pub_ = self.create_publisher(
                AlternativeArray,
                'alternatives',
                10)

    def get_alternatives_cb(self, goal_handle):
        gap = goal_handle.request.gap
        maxresult = goal_handle.request.maxresult # Bounded rationality ;)
        self.get_logger().info(f'Getting up to {maxresult} alternatives matching gap: {gap}')

        try:
            matches = self.match_templates(gap.templates, maxresult, goal_handle)
        except ValueError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return GetAlternatives.Result()

        alternatives = AlternativeArray(alternatives=matches)
        self.pub_.publish(alternatives)

        goal_handle.succeed()
        self.get_logger().info(f'Succeeded with alternatives: {alternatives}')
        return GetAlternatives.Result(alternatives=alternatives)

    def match_templates(self, templates, maxresult, goal_handle):
        unique_matches = set()
        for template in templates:
            if len(unique_matches) >= maxresult:
                break

            # TODO: ignore already matched alternatives in successive queries
            matches = self._match_template(template.query, maxresult - len(unique_matches))
            if len(matches) == 0:
                self.get_logger().warning(f'No bindings for "A" in answers to query: "{template.query}"')
            else:
                unique_matches.update(matches)
                goal_handle.publish_feedback(GetAlternatives.Feedback(num_matched=len(unique_matches)))

        return [Alternative(id=match) for match in unique_matches]

    def _match_template(self, query, max_remaining):
        """
        :raises ValueError: If a query could not be answered or answer was
            missing a binding for the alternative variable `A`.
        """
        self.get_logger().info(f'Matching up to {max_remaining} alternatives matching Prolog query: {query}')
        request = PrologQuery.Request()

        request.query.clause = query
        request.maxresult = max_remaining

        response = self.call_service(self.prolog_client_, request)
        if response is None or not response.success or len(response.answers) == 0:
            raise ValueError(f'Failed to answer query: "{request.query.clause}"')

        alternatives = []
        for answer in response.answers:
            for binding in answer.bindings:
                if binding.key == "A":
                    alternatives.append(binding.value)

        return alternatives

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

    node = GetAlternativesPrologActionServer()

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
