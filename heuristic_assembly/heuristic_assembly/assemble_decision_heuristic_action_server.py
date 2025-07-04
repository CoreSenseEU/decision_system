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
import os
import xml.etree.ElementTree as ET
import yaml

import rclpy
from rclpy.action import ActionServer
from ament_index_python.packages import get_package_share_directory

from decision_msgs.action import AssembleDecisionHeuristic
from prolog_kb.prolog_interface import PrologInterface


TEST_OUTPUT_FILE = os.path.expanduser('~/thesis/src/out.txt')


class AssembleDecisionHeuristicActionServer(PrologInterface):
    """
    An action to assemble a new decision heuristic.

    :param working_directory: The working directory where new heuristics and
        configuration files should be saved. Defaults to the current working directory.

    """
    def __init__(self):
        super().__init__('assemble_decision_heuristic_action_server')
        self.get_logger().info('Starting ASSEMBLE_DECISION_HEURISTIC action server')

        self.declare_parameter('working_directory', os.getcwd())

        self.action_server_ = ActionServer(
                self,
                AssembleDecisionHeuristic,
                'AssembleDecisionHeuristic',
                self.assemble_cb)

    def assemble_cb(self, goal_handle):
        heuristic_dir = os.path.join(self.get_parameter('working_directory').value, 'heuristics')
        os.makedirs(heuristic_dir, exist_ok=True)

        gap = goal_handle.request.gap_id
        heuristic_name = f'DecideOnGap_{gap}'

        try:
            pipeline = self.assemble_with_gap(gap)
            xml = self.write_to_xml(gap, pipeline, heuristic_dir, heuristic_name)
            yaml = self.write_to_yaml(gap, pipeline, heuristic_dir, heuristic_name)
        except Exception as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return AssembleDecisionHeuristic.Result()

        goal_handle.succeed()
        return AssembleDecisionHeuristic.Result(heuristic_file=xml, 
                                                params_file=yaml,
                                                entry_point=heuristic_name)

    def assemble_with_gap(self, gap):
        # TODO: update this when the understanding core is in better condition
        file_path = TEST_OUTPUT_FILE
        self.get_logger().warn(f'Using mock understanding output from {file_path}')

        with open(file_path, 'r') as f:
            lines = f.readlines()

        answer = ''
        for line in lines:
            if 'SZS answers Tuple' in line:
                answer = line
                break
        if answer == '':
            raise RuntimeError(f'No answer detected in the Vampire output file {file_path}')

        first_heuristic = answer.split('|')[0].split('exert(') #)
        engines = [part[:part.find('_engine')] for part in first_heuristic if '_engine' in part]

        ### TODO: remove this hack
        #   The first example was (conveniently) elimination-by-aspects (with preference ordering)
        engines += ['update_alternatives_elim', 'update_cues_iter_one']
        ###

        return engines

    def write_to_yaml(self, gap, pipeline, heuristic_dir, heuristic_name):
        params = {}
        for engine in pipeline:
            config_path = os.path.join(get_package_share_directory('heuristic_assembly'), self._get_ros_params(engine))
            ros_node = self._get_ros_node(engine)
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Use the provided name for setting the parameters
            # TODO: maybe switch to using a namespace instead, then these
            #   parameter files wouldn't need to hardcode node names/assume the
            #   name name space if there are multiple nodes
            if '/**' in config.keys():
                config[ros_node] = config.pop('/**')
            elif len(config.keys()) == 1:
                config[ros_node] = config.popitem()[1]
            else:
                raise RuntimeError(f"Unable to identify target node {ros_node} for {engine} engine in config: {config}")

            if len(config[ros_node]['ros__parameters']) == 0:
                self.get_logger().debug(f"No parameters set for {engine}. Skipping")
                continue

            params.update(config)

        yaml_path = os.path.join(heuristic_dir, heuristic_name + '.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(params, f, default_flow_style=False)
        self.assertz(f"config_of('{heuristic_name}', '{gap}')")

        self.get_logger().info(f'Successfully wrote config to: {yaml_path}')
        self.get_logger().debug(yaml.dump(params))
        return yaml_path

    def write_to_xml(self, gap, pipeline, heuristic_dir, heuristic_name):
        root = ET.Element('root', attrib={'BTCPP_format' : "4"})
        main = ET.SubElement(root, 'BehaviorTree', attrib={'ID': heuristic_name})
        main.append(ET.Element('SubTree', attrib={'ID': 'MakeSureGapClosed',
                                                  '_autoremap': 'true', 
                                                  'gap': '{@payload}',
                                                  'choice_query': '{@return_message}'}))

        for engine in pipeline:
            meta = os.path.join(get_package_share_directory('heuristic_assembly'), self._get_meta(engine))
            ros_node = self._get_ros_node(engine)
            behavior_tree = ET.parse(meta)
            for node in behavior_tree.iter():
                if node.get('action_name') == '':
                    node.set('action_name', ros_node + '/' + node.tag)
            self._append_all_bts(root, behavior_tree)

        decide_structure = os.path.join(get_package_share_directory('heuristic_assembly'), 'behavior_trees/decide_structure.xml')
        self._append_all_bts(root, ET.parse(decide_structure))

        writer = ET.ElementTree(root)
        xml_path = os.path.join(heuristic_dir, f'{gap}.xml')
        writer.write(xml_path)
        self.assertz(f"heuristic_of('{heuristic_name}', '{gap}')")

        self.get_logger().info(f'Successfully wrote heuristic to: {xml_path}')
        self.get_logger().debug(ET.tostring(root))

        return xml_path

    def _append_all_bts(self, parent, tree):
        parent.extend(tree.iter('BehaviorTree'))

        # Also add includes (assume they are all ROS flavored)
        parent.extend(tree.iter('include'))

    def _get_ros_node(self, engine):
        answers = self.query(f'engine({engine}), has_ros_node({engine}, N)', maxresult=1) 

        if len(answers) == 0:
            raise RuntimeError(f'Cound not find ROS nodes for {engine} engine')
        if len(answers) > 1:
            raise RuntimeError(f'Found multiple ROS nodes for {engine} engine: {[ans["N"] for ans in answers]}')
            
        return answers[0]["N"]

    def _get_ros_params(self, engine):
        answers = self.query(f'engine({engine}), uses_ros_params({engine}, C)', maxresult=1) 

        if len(answers) == 0:
            raise RuntimeError(f'Cound not find ROS parameter config for {engine} engine')
        if len(answers) > 1:
            raise RuntimeError(f'Found multiple ROS parameter configs for {engine} engine: {[ans["N"] for ans in answers]}')
            
        return answers[0]["C"]

    def _get_meta(self, engine):
        answers = self.query(f'engine({engine}), has_meta({engine}, M)', maxresult=1) 

        if len(answers) == 0:
            raise RuntimeError(f'Cound not find meta for {engine} engine')
        if len(answers) > 1:
            raise RuntimeError(f'Found multiple metas for {engine} engine: {[ans["M"] for ans in answers]}')
            
        return answers[0]["M"]



def main(args=None):
    rclpy.init(args=args)

    node = AssembleDecisionHeuristicActionServer()

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
