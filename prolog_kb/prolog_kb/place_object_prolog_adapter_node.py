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

from krr_mirte_skills_msgs.srv import PlaceObject

from prolog_kb.prolog_interface import PrologInterface


class PlaceObjectPrologAdapter(PrologInterface):
    """
    An adapter that intercepts the /place_object service, and publishes the
    results of the service call into a prolog knowledge base.
    """
    def __init__(self):
        super().__init__('place_object_prolog_adapter')

        self.srv_place_object_ = self.create_service(
                PlaceObject,
                'place_object',
                self.place_object_cb,
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_place_object_ = self.create_client(
                PlaceObject,
                'shadow/place_object',
                callback_group=MutuallyExclusiveCallbackGroup())

    def place_object_cb(self, request, result):
        self.get_logger().info('Forwarding service request to krr_mirte_skills: /place_object')
        result = self.call_service_or(self.client_place_object_, request, result)
        if not result.success:
            self.get_logger().warning('Service request to /place_object failed, no information to capture.')
            return result

        self.retract("is_held(_)")

        # TODO: should this be smarter and assign the newest pose to this object
        #  - Or is that the responsibility of some other node/logic?

        self.get_logger().info('Captured information from service request: /place_object')
        return result


def main(args=None):
    rclpy.init(args=args)

    node = PlaceObjectPrologAdapter()

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
