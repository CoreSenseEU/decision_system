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

from krr_mirte_skills_msgs.srv import GetObjectsInRoom
from decision_msgs.srv import CreatePose

from prolog_kb.prolog_interface import PrologInterface


class GetObjectsInRoomPrologAdapter(PrologInterface):
    """
    An adapter that intercepts the /get_objects_in_room service, and publishes the
    results of the service call into a prolog knowledge base.
    """
    def __init__(self):
        super().__init__('get_objects_in_room')

        self.srv_get_objects_in_room_ = self.create_service(
                GetObjectsInRoom,
                'get_objects_in_room',
                self.get_objects_in_room_cb,
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_get_objects_in_room_ = self.create_client(
                GetObjectsInRoom,
                'shadow/get_objects_in_room',
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_create_pose_ = self.create_client(
                CreatePose,
                'create_pose',
                callback_group=MutuallyExclusiveCallbackGroup())

    def get_objects_in_room_cb(self, request, result):
        self.get_logger().info('Forwarding service request to krr_mirte_skills: /get_objects_in_room')
        result = self.call_service_or(self.client_get_objects_in_room_, request, result)
        if not result.success:
            self.get_logger().warning('Service request to /get_objects_in_room failed, no information to capture.')
            return result
        
        assertions = []
        for pose in result.room_object_poses:
            pose_id = self._create_pose(pose)
            if pose_id is None or len(self._find_objects_with_pose(pose_id)) > 1:
                result.success = False
                return result
            assertions.append(f"has_pose('', {pose_id})")

        for door in result.doorway_object_poses:
            for pose in door.objects_in_doorway:
                pose_id = self._create_pose(pose)
                if pose_id is None:
                    result.success = False
                    return result
                matching_objects = self._find_objects_with_pose(pose_id)
                if len(matching_objects) > 1:
                    result.success = False
                    return result

                if len(matching_objects) == 1:
                    assertions.append(f"in_doorway({door.which_doorway}, {matching_objects[0]})")
                else:
                    assertions.append(f"in_doorway({door.which_doorway}, '')")
                    assertions.append(f"has_pose('', {pose_id})")

        for assertion in assertions:
            self.assertz(assertion)

        self.get_logger().info('Captured information from service request: /get_objects_in_room')
        return result

    def _create_pose(self, pose):
        new_pose_result = self.call_service_or(
                self.client_create_pose_, 
                CreatePose.Request(pose=pose), 
                CreatePose.Response())
        if not new_pose_result.success:
            self.get_logger().error(f'Failed to create a new pose for object with pose {pose}')
            return None
        return new_pose_result.id

    def _find_objects_with_pose(self, pose_id):
        answers = self.query(f'object(O), has_pose(O, {pose_id})')
        matching_objects = []
        for answer in answers:
            matching_objects.append(answer["O"])
            self.get_logger().warning(f'Detected object \'{answer["O"]}\' with pose {pose_id} again.')
        if len(matching_objects) > 1:
            self.get_logger().error(
                    f'Found multiple objects: {matching_objects} '
                    f'with the same pose: {pose_id}')
        return matching_objects


def main(args=None):
    rclpy.init(args=args)

    node = GetObjectsInRoomPrologAdapter()

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
