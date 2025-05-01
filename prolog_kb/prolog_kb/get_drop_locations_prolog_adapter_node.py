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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from krr_mirte_skills_msgs.srv import GetDropLocations

from prolog_kb.pose import Pose
from prolog_kb.prolog_adapter import PrologAdapter


class GetDropLocationsPrologAdapter(PrologAdapter):
    """
    An adapter that intercepts the /get_drop_locations service, and publishes the
    results of the service call into a prolog knowledge base.
    """
    _drop_tag = 0

    def __init__(self):
        super().__init__('get_drop_locations')

        self.srv_get_drop_locations_ = self.create_service(
                GetDropLocations,
                'get_drop_locations',
                self.get_drop_locations_cb,
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_get_drop_locations_ = self.create_client(
                GetDropLocations,
                'shadow/get_drop_locations',
                callback_group=MutuallyExclusiveCallbackGroup())

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
            self.assertz(assertion)

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
            self.assertz(f"pose({pose_id})")
            self.assertz(f"has_coordinates_7d({pose_id}, "
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
        answers = self.query(f"has_coordinates_7d(P, "
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
