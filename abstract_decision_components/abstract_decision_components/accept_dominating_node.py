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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from decision_interfaces.srv import AcceptDominating
import accept


class AcceptDominatingNode(Node):
    def __init__(self):
        super().__init__('accept_dominating_node')
        self.get_logger().info('Starting ACCEPT node with policy: accept_dominating')

        # All services can be called in parallel
        cb_group = ReentrantCallbackGroup()

        self.srv_dominating = self.create_service(
            AcceptDominating,
            'accept_dominating',
            self.accept_dominating_cb,
            callback_group=cb_group)

    def accept_dominating_cb(self, request, response):
        """
        Accepts a choice if the scores of all chosen alternatives are strictly
        greater than the best unchosen alternative for each requested axis.
        """
        response.reason, response.success = accept.satisficing(request.choice, request.judgments, axies=request.axies)

        if response.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {request.choice} with policy: accept_dominating, "{request.axies}"')
        return response
 

def main(args=None):
    rclpy.init(args=args)

    node = AcceptDominatingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
