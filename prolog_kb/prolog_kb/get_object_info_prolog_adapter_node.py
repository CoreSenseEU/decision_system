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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from krr_mirte_skills_msgs.srv import GetObjectInfo
from decision_msgs.msg import PrologClause
from decision_msgs.srv import PrologQuery


class GetObjectInfoPrologAdapter(Node):
    """
    An adapter that intercepts the /get_object_info service, and publishes the
    results of the service call into a prolog knowledge base.
    """
    def __init__(self):
        super().__init__('get_object_info_prolog_adapter')
        self.get_logger().info('Starting prolog knowledge base adapter for service: /get_object_info')

        self.srv_get_object_info_ = self.create_service(
                GetObjectInfo,
                'get_object_info',
                self.get_object_info_cb,
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_get_object_info_ = self.create_client(
                GetObjectInfo,
                'shadow/get_object_info',
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_query_ = self.create_client(
                PrologQuery,
                'query',
                callback_group=MutuallyExclusiveCallbackGroup())

        self.pub_assert_ = self.create_publisher(
                PrologClause,
                'assert',
                self.assert_cb,
                10)

        self.pub_retract_ = self.create_publisher(
                PrologClause,
                'retract',
                self.retract_cb,
                10)

    def get_object_info_cb(self, request, result):
        result = self.call_service(self.client_get_object_info_, request)
        if not result.success:
            return result

        try:
            object_id = self._query("is_held(O)", maxresult=1)[0]["O"]
        except Exception as e:
            self.get_logger().error(
                    "Recieved the following error while attempting to"
                    f" retrieve the held object id:\n{str(e)}")
            return result

        # Object type and attribute exist
        self._assert(f"object_type({result.object_type})")
        self._assert(f"attribute({result.attribute})")

        # The object which is held has that type and attribute
        self._assert(f"has_type({object_id}, {result.object_type})")
        if result.attribute != '':
            self._assert(f"has_attribute({object_id}, {result.attribute})")

        return result

    def _query(self, clause, maxresult=-1):
        request = PrologQuery.Request()
        request.query.clause = clause
        request.maxresult = maxresult
        result = self.call_service(self.client_query_, request)

        answers = []
        for answer in result.answers:
            answers.append({binding.key: binding.value for binding in answer.bindings})
        return answers

    def _assert(self, clause):
        self.pub_assert_.publish(PrologClause(clause=clause))

    def _retract(self, clause):
        self.pub_retract_.publish(PrologClause(clause=clause))

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

    node = GetObjectInfoPrologAdapter()

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
