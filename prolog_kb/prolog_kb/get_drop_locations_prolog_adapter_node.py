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
from decision_msgs.srv import CreatePose

from prolog_kb.prolog_interface import PrologInterface


class GetDropLocationsPrologAdapter(PrologInterface):
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

        self.client_create_pose_ = self.create_client(
                CreatePose,
                'create_pose',
                callback_group=MutuallyExclusiveCallbackGroup())

    @classmethod
    def _get_next_drop(cls):
        drop_id = f'drop_{cls._drop_tag}'
        cls._drop_tag += 1
        return drop_id

    def get_drop_locations_cb(self, request, result):
        self.get_logger().info('Forwarding service request to krr_mirte_skills: /get_drop_locations')
        result = self.call_service_or(self.client_get_drop_locations_, request, result)
        if not result.success:
            self.get_logger().warning('Service request to /get_drop_locations failed, no information to capture.')
            return result

        assertions = []
        for drop in result.drop_locations:
            new_pose_result = self.call_service_or(
                    self.client_create_pose_, 
                    CreatePose.Request(pose=drop.drop_pose), 
                    CreatePose.Response())
            if not new_pose_result.success:
                self.get_logger().error(f'Failed to create a new pose for drop with pose {drop.drop_pose}')
                result.success = False
                return result
            
            # TODO: move to a new file (similar to PoseFactory.get_next_pose())
            drop_id = GetDropLocationsPrologAdapter._get_next_drop()

            assertions.append(f"drop({drop_id})")
            assertions.append(f"has_drop_type({drop_id}, {drop.type.data})")
            assertions.append(f"has_pose({drop_id}, {new_pose_result.id})")

        for assertion in assertions:
            self.assertz(assertion)

        self.get_logger().info('Captured information from service request: /get_drop_locations')
        return result


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
