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
import re

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from decision_msgs.msg import PrologClause, PrologAnswer, PrologBinding
from decision_msgs.srv import PrologQuery

from pyswip import Prolog
from pyswip.prolog import PrologError


class PrologServer(Node):
    """
    A service to query and update a prolog knowledge base.

    :param knowledge_base: A path to an inital prolog knowledge base to consult.
    """
    def __init__(self):
        super().__init__('prolog_kb_server')
        self.get_logger().info('Starting prolog knowledge base server')

        self.add_on_set_parameters_callback(self._reload_kb)
        self.declare_parameter('knowledge_base', Parameter.Type.STRING)

        self.srv_ = self.create_service(
                PrologQuery,
                'query',
                self.prolog_query_cb
                )

        self.sub_assert_ = self.create_subscription(
                PrologClause,
                'assert',
                self.assert_cb,
                10)

        self.sub_retract_ = self.create_subscription(
                PrologClause,
                'retract',
                self.retract_cb,
                10)

        self.kb_ = None
        self.prolog = Prolog()

    def prolog_query_cb(self, request, result):
        result.success = False
        if len(request.query.clause) < 1:
            self.get_logger().error('Recieved empty query.')
            return result

        self.get_logger().info(f'Submitting prolog query to knowledge base: "{request.query.clause}"')
        try:
            answers = list(self.prolog.query(request.query.clause,
                                             maxresult=request.maxresult,
                                             catcherrors=True))
        except PrologError as e:
            self.get_logger().error(str(e))
            return result

        if len(answers) == 0:
            self.get_logger().info('Answer: False')
            return result
        elif answers[0] == {}:      # Asked a True/False question
            result.success = True
            self.get_logger().info('Answer: True')
        else:                       # Asked a question with variables
            answer_strs = []
            for answer in answers:
                answer_strs.append(str(answer))
                bindings = [PrologBinding(key=key, value=str(value)) for key, value in answer.items()]
                result.answers.append(PrologAnswer(bindings=bindings))
            self.get_logger().info(f'Answers: [{", ".join(answer_strs)}]')
            result.success = True
        return result

    def assert_cb(self, msg):
        if len(msg.clause) < 1:
            self.get_logger().error('Recieved empty clause.')

        try:
            self.prolog.assertz(msg.clause, catcherrors=True)
            self.get_logger().info(f'Added clause to end of knowledge base: "{msg.clause}"')
        except PrologError as e:
            self.get_logger().error(str(e))

    def retract_cb(self, msg):
        if len(msg.clause) < 1:
            self.get_logger().error('Recieved empty clause.')

        try:
            self.prolog.retractall(msg.clause, catcherrors=True)
            self.get_logger().info(f'Retracted clause(s) from knowledge base: "{msg.clause}"')
        except PrologError as e:
            self.get_logger().error(str(e))

    def _reload_kb(self, parameters):
        for p in parameters:
            if p.name == 'knowledge_base':
                self.kb_ = str(p.value)
                self.get_logger().info(f'Reloading knowledge base with {self.kb_}')
                self.prolog = Prolog()
                self.prolog.consult(self.kb_, relative_to=__file__)
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def _get_indicators(self, clause):
        matches = re.findall(r'\s?(\w+?)\((.*?)\)', clause)
        for match in matches:
            if match[1] == '':
                arity = 0
            else:
                arity = match[1].count(',') + 1
            yield match[0], arity


def main(args=None):
    rclpy.init(args=args)

    node = PrologServer()

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
