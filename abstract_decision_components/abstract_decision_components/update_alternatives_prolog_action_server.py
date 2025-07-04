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

from decision_msgs.action import UpdateAlternatives
from decision_msgs.srv import PrologQuery


class UpdateAlternativesActionServer(Node):
    """
    An action server to update the choice set from a set of available alternatives in the prolog KB.

    :param keep_policy: A policy to determine what alternatives to keep from the previous iteration.
        - `'all'`: keep all alternatives from the previous iteration.
        - `'none'`: do not keep any alternatives from the previous iteration.
        - `'taken'`: keep only the alternatives that were taken in the previous iteration.
        Defaults to 'taken'

    :param capacity: The maximum number of alternatives to consider in the next iteration.
        A value of `0` is interpreted as no maximum. Defaults to `0`.

    """

    def __init__(self):
        super().__init__('update_alternatives_prolog_action_server')
        self.get_logger().info('Starting UPDATE_ALTERNATIVES action server')

        self.declare_parameter('keep_policy', 'taken')
        self.declare_parameter('capacity', 0)

        self.action_server_ = ActionServer(
                self,
                UpdateAlternatives,
                'UpdateAlternatives',
                self.update_alternatives_cb)

        self.prolog_client_ = self.create_client(
                PrologQuery,
                'query',
                callback_group = MutuallyExclusiveCallbackGroup())

    def update_alternatives_cb(self, goal_handle):
        cap = self.get_parameter('capacity').value
        policy = self.get_parameter('policy').value
        self.get_logger().info(f'Updating choice set with up to {cap} alternatives with policy: {policy}')

        if policy == 'taken':
            choice_set = goal_handle.request.previous_taken
        elif policy == 'all':
            choice_set = goal_handle.request.previous_choice_set
        else:
            choice_set = []

        # refill
        query = "alternative_of(A, '_G'), \+ fetched_for(A, '_G'), assertz((fetched_for(A, '_G')))"
        query.replace('_G', goal_handle.request.gap)
        try:
            if cap > 0:
                choice_set.append(self._get_available(query, cap - len(choice_set)))
            else:
                choice_set.append(self._get_available(query, -1))
        except ValueError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return UpdateAlternatives.Result()

        goal_handle.succeed()
        self.get_logger().info(f'Succeeded with alternatives: {choice_set}')
        return UpdateAlternatives.Result(choice_set=choice_set)

    def _get_available(self, query, maxresult):
        """
        :raises ValueError: If a query could not be answered or answer was
            missing a binding for the alternative variable `_A`.
        """
        request = PrologQuery.Request(query=query, maxresult=maxresult)

        response = self.call_service(self.prolog_client_, request)
        if response is None:
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

    node = UpdateAlternativesActionServer()

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
