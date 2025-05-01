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


class PrologAdapter(Node):
    """
    An adapter that a node can inherit to facilitate communicating with a
    prolog knowledge base.
    """
    def __init__(self, name):
        super().__init__(f'{name}_prolog_adapter')
        self.get_logger().info(f'Starting prolog knowledge base adapter for service: /{name}')

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

    def query(self, clause, maxresult=-1):
        """
        :raises TimeoutError: if the query does not become available or times out.
        """
        request = PrologQuery.Request()
        request.query.clause = clause
        request.maxresult = maxresult
        result = self.call_service(self.client_query_, request)
        return [{b.key: b.value for b in ans.bindings} for ans in result.answers]

    def assertz(self, clause):
        self.pub_assert_.publish(PrologClause(clause=clause))

    def retract(self, clause):
        self.pub_retract_.publish(PrologClause(clause=clause))

    def call_service(self, cli, request):
        """
        Adapted from https://github.com/kas-lab/krr_mirte_skills/blob/main/krr_mirte_skills/krr_mirte_skills/get_object_info.py
        :raises TimeoutError: if the service does not become available or times out.
        """
        if not cli.wait_for_service(timeout_sec=5.0):
            raise TimeoutError('service not available {}'.format(cli.srv_name))
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done():
            raise TimeoutError('Future not completed {}'.format(cli.srv_name))
        return future.result()


