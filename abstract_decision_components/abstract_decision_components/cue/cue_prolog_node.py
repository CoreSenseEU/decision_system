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
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from decision_msgs.msg import Preference
from decision_msgs.srv import AssessAlternatives

from pyswip import Prolog


class CuePrologNode(Node):
    """
    A service to queries a prolog knowledge base to answer questions about alternatives.
    This assumes answers about alternatives are independent.

    :param query: A prolog query to submit to the database where the variable `A`
        is used for input alternatives and `V` is used for the desired value.
    :param knowledge_base: A path to the prolog knowledge base.
    """
    def __init__(self):
        super().__init__('cue_prolog_service')
        self.get_logger().info('Starting CUE node which queries a prolog knowledge base')

        self.declare_parameter('query', Parameter.Type.STRING) # prolog query referring to the alternative as A
        self.add_on_set_parameters_callback(self._reload_kb)
        self.declare_parameter('knowledge_base', Parameter.Type.STRING) # knowledge base path

        self.srv_ = self.create_service(
                AssessAlternatives,
                'assess',
                self.query_prolog_cb
                )

        self.kb_ = None
        self.prolog = Prolog()

    def query_prolog_cb(self, request, result):
        if len(request.alternatives) < 1:
            self.get_logger().error('No alternatives to assess')
            return result

        query = self.get_parameter('query').value
        result.preferences = [Preference(alternative=a, score=self.assess_one(a)) for a in request.alternatives]
        self.get_logger().info(f'Assessing alternatives {request.alternatives} with policy: cue_prolog "{query}"')
        return result

    def assess_one(self, alternative):
        query = self.get_parameter('query').value

        self.prolog.assertz(f"alternative('{alternative.id}')")
        answer = self.prolog.query(f'{query}, alternative(A)', maxresult=1)
        score = None
        for res in answer:
            score = float(res["V"])
        self.prolog.retract(f"alternative('{alternative.id}')")
        
        if score is None:
            self.get_logger().error(f'No answers to query "{query}"')
        return score

    def _reload_kb(self, parameters):
        for p in parameters:
            if p.name == 'knowledge_base':
                self.kb_ = str(p.value)
                self.get_logger().info(f'Reloading knowledge base with {self.kb_}')
                self.prolog = Prolog()
                self.prolog.consult(self.kb_, relative_to=__file__)
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = CuePrologNode()

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
