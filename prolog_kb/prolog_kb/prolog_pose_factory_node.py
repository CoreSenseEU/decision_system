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

from decision_msgs.srv import CreatePose

from prolog_kb.prolog_interface import PrologInterface


class PrologPoseFactoryNode(PrologInterface):
    """Add a new pose to the Prolog knowledge base if it doesn't already exist.
    """
    _tag = 0

    def __init__(self):
        super().__init__('create_pose')

        self.srv_get_drop_locations_ = self.create_service(
                CreatePose,
                'create_pose',
                self.create_pose_cb)

    @classmethod
    def _get_next_pose(cls):
        name = f'pose_{cls._tag}'
        cls._tag += 1
        return name

    def create_pose_cb(self, request, result):
        coordinates = (
                f"{request.pose.position.x}, "
                f"{request.pose.position.y}, "
                f"{request.pose.position.z}, "
                f"{request.pose.orientation.x}, "
                f"{request.pose.orientation.y}, "
                f"{request.pose.orientation.z}, "
                f"{request.pose.orientation.w}"
                )

        # TODO: Be smarter about this, and compare using a float threshold
        answers = self.query("has_coordinates_7d(P, " + coordinates + "), pose(P)", maxresult=1)
        if len(answers) > 1:
            self.get_logger().error(f'Found duplicate poses: {[ans["P"] for ans in answers]}')
            return result

        if len(answers) == 1:
            pose_id = answers[0]["P"]
            self.get_logger().warning(
                    f'A matching pose ({pose_id}) already exists with coordinates: [{coordinates}]')
        else:
            pose_id = self._get_next_pose()
            self.assertz(f"pose({pose_id})")
            self.assertz(f"has_coordinates_7d({pose_id}, {coordinates})")
            self.get_logger().info(f'Created new pose ({pose_id}) with coordinates: [{coordinates}]')

        result.id = pose_id
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    node = PrologPoseFactoryNode()

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
