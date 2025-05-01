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

from krr_mirte_skills_msgs.srv import GetObjectInfo

from prolog_kb.prolog_interface import PrologInterface


class GetObjectInfoPrologAdapter(PrologInterface):
    """
    An adapter that intercepts the /get_object_info service, and publishes the
    results of the service call into a prolog knowledge base.
    """
    def __init__(self):
        super().__init__('get_object_info')

        self.srv_get_object_info_ = self.create_service(
                GetObjectInfo,
                'get_object_info',
                self.get_object_info_cb,
                callback_group=MutuallyExclusiveCallbackGroup())

        self.client_get_object_info_ = self.create_client(
                GetObjectInfo,
                'shadow/get_object_info',
                callback_group=MutuallyExclusiveCallbackGroup())

    def get_object_info_cb(self, request, result):
        self.get_logger().info('Forwarding service request to krr_mirte_skills: /get_object_info')
        result = self.call_service_or(self.client_get_object_info_, request, result) 
        if not result.success:
            self.get_logger().warning('Service request to /get_object_info failed, no information to capture.')
            return result


        answers = self.query("is_held(O)", maxresult=1)
        if len(answers) != 1:
            result.success = False
            return result
        object_id = answers[0]["O"]

        # Object type and attribute exist
        self.assertz(f"object_type({result.object_type})")
        self.assertz(f"attribute({result.attribute})")

        # The object which is held has that type and attribute
        self.assertz(f"has_type({object_id}, {result.object_type})")
        if result.attribute != '':
            self.assertz(f"has_attribute({object_id}, {result.attribute})")

        self.get_logger().info('Captured information from service request: /get_object_info')
        return result


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
