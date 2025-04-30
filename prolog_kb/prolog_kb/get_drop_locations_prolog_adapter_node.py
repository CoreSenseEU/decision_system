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

from krr_mirte_skills_msgs.srv import GetDropLocations
from decision_msgs.msg import PrologClause
from decision_msgs.srv import PrologQuery

from prolog_kb.pose import Pose


class GetDropLocationsPrologAdapter(Node):
    """
    An adapter that intercepts the /get_drop_locations service, and publishes the
    results of the service call into a prolog knowledge base.
    """
    _drop_tag = 0

    def __init__(self):
        super().__init__('get_drop_locations_adapter')
        self.get_logger().info('Starting prolog knowledge base adapter for service: /get_drop_locations')

        self.srv_get_drop_locations_ = self.create_service(
                GetDropLocations,
                'get_drop_locations',
                self.get_drop_locations_cb,
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_get_drop_locations_ = self.create_client(
                GetDropLocations,
                'shadow/get_drop_locations',
                callback_group=MutuallyExclusiveCallbackGroup())

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

    def get_drop_locations_cb(self, request, result):
        result = self.call_service(self.client_get_drop_locations_, request)
        if result is None:
            result = GetDropLocations.Response()
            return result
        if not result.success:
            return result

        assertions = []
        for drop in result.drop_locations:
            try:
                # TODO: poses may be created in the knowledge base even if others fail
                #   This leaves the KB in a different state than it started, but maybe that's okay...
                pose_id = self._create_new_anonymous_pose(drop.drop_pose)
            except TimeoutError as e:
                self.get_logger().error(str(e))
                result.success = False
                return result
            
            # TODO: move to a new file (similar to Pose.get_next())
            drop_id = GetDropLocationsPrologAdapter._get_next_drop()

            assertions.append(f"drop({drop_id})")
            assertions.append(f"has_drop_type({drop_id}, {drop.type.data})")
            assertions.append(f"has_pose({drop_id}, {pose_id})")

        for assertion in assertions:
            self._assert(assertion)

        return result

    @classmethod
    def _get_next_drop(cls):
        drop_id = f'drop_{cls._drop_tag}'
        cls._drop_tag += 1
        return drop_id

    def _create_new_anonymous_pose(self, pose):
        """
        :raises TimeoutError: if the query does not become available or times out.
        """
        # TODO: Move this logic to another python file, it is duplicated in get_objects_in_room_prolog_adapter_node.py
        duplicate_id = self._find_duplicate_pose(pose)
        if duplicate_id is not None:
            self.get_logger().info(f'A matching pose ({duplicate_id}) already exists: {pose}')
            pose_id = duplicate_id
        else:
            pose_id = Pose.get_next()
            # Create new pose
            self._assert(f"pose({pose_id})")
            self._assert(f"has_coordinates_7d({pose_id}, "
                         f"{pose.position.x}, "
                         f"{pose.position.y}, "
                         f"{pose.position.z}, "
                         f"{pose.orientation.x}, "
                         f"{pose.orientation.y}, "
                         f"{pose.orientation.z}, "
                         f"{pose.orientation.w}"
                         ")")
        return pose_id

    def _find_duplicate_pose(self, pose):
        """
        :raises TimeoutError: if the query does not become available or times out.
        """
        # TODO: Move this logic to another python file, it is duplicated in get_objects_in_room_prolog_adapter_node.py
        # TODO: evaluate within some tolerance for floating point errors
        answers = self._query(f"has_coordinates_7d(P, "
                             f"{pose.position.x}, "
                             f"{pose.position.y}, "
                             f"{pose.position.z}, "
                             f"{pose.orientation.x}, "
                             f"{pose.orientation.y}, "
                             f"{pose.orientation.z}, "
                             f"{pose.orientation.w}"
                             "), pose(P)", maxresult=1)
        if len(answers) == 1:
            return answers[0]["P"]
        return None

    def _query(self, clause, maxresult=-1):
        """
        :raises TimeoutError: if the query does not become available or times out.
        """
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


def main(args=None):
    rclpy.init(args=args)

    node = GetDropLocationsPrologAdapter()

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
