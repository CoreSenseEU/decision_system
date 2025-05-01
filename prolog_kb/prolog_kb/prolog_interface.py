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

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from decision_msgs.msg import PrologClause
from decision_msgs.srv import PrologQuery


class PrologInterface(Node):
    """
    An interface that a node can inherit to facilitate communicating with a
    prolog knowledge base.
    """
    def __init__(self, name):
        """
        :param name: The ROS service that this interface provides.
        """
        super().__init__(f'{name}_prolog_interface')
        self.get_logger().info(f'Starting prolog knowledge base interface for service: /{name}')

        self.client_query_ = self.create_client(
                PrologQuery,
                'query',
                callback_group=MutuallyExclusiveCallbackGroup())

        self.pub_assert_ = self.create_publisher(
                PrologClause,
                'assert',
                10)

        self.pub_retract_ = self.create_publisher(
                PrologClause,
                'retract',
                10)

    def query(self, goal: str, maxresult: int = -1):
        """Query a goal from the knowledge base.

        :param goal: The prolog goal to query
        :param maxresult: The maximum number of desired results. Returns all 
            results if this is below 0, defaults to -1

        :return: A list of answers from the knowledge base where each answer is
            is a dictionary of variable bindings from the query goal.
        """
        request = PrologQuery.Request()
        request.query.clause = goal
        request.maxresult = maxresult
        result = self.call_service_or(self.client_query_, request, PrologQuery.Response())
        return [{b.key: b.value for b in ans.bindings} for ans in result.answers]

    def assertz(self, clause: str):
        """Assert a clause at the end of the knowledge base.

        :param clause: A clause to assert
        """
        self.pub_assert_.publish(PrologClause(clause=clause))

    def retract(self, clause: str):
        """Retract all matching clauses from the knowledge base.

        :param clause: A clause to match on
        """
        self.pub_retract_.publish(PrologClause(clause=clause))

    def call_service_or(self, client, request, alternative_value=None):
        """Call a service and wait for it to finish or timeout.
        Adapted from https://github.com/kas-lab/krr_mirte_skills/blob/main/krr_mirte_skills/krr_mirte_skills/get_object_info.py

        :param client: The ROS client to use to call this service
        :param request: The service request 
        :param alternative_value: An value to return if the future is done but
            does not have a result. Optional.

        :return: The result of the service call, or the alternative value.
        """
        # TODO: replace magic numbers
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('service not available {}'.format(client.srv_name))
            return alternative_value
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error('Future not completed {}'.format(client.srv_name))
            return alternative_value
        
        if future.result() is None:
            return alternative_value
        return future.result()


