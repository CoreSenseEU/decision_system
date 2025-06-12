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

from abstract_decision_components.accept.accept import compare_size
from abstract_decision_components.accept.accept_action_server import AcceptActionServer


class AcceptSizeActionServer(AcceptActionServer):
    """Accepts a choice based on the number of chosen alternatives.

    :param n: An integer size to compare to. Defaults to 1
    :param relation: A relational operator with the rhs of ``choice`` on the
        right hand size and ``rhs`` on the left. Valid relation operators are:
            `<`, `>`, `<=`, `>=`, `=`, and `!=`.
        Defaults to '='
    """
    def __init__(self):
        super().__init__('size')
        self.declare_parameter('n', 1)
        self.declare_parameter('relation', '=')

    def accept(self, choice):
        n = self.get_parameter('n').value
        relation = self.get_parameter('relation').value
        return compare_size(choice.chosen, n, relation=relation)


def main(args=None):
    rclpy.init(args=args)

    node = AcceptSizeActionServer()

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
