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
import subprocess

import rclpy
from rclpy.action import ActionServer
from ament_index_python.packages import get_package_share_directory

from decision_msgs.action import AssembleDecisionHeuristic
from prolog_kb.prolog_interface import PrologInterface


GAP_FILE = os.path.join(os.environ['TPTP'], 'decision-logic/tff/tests/test-decision-questions.tff')

VAMPIRE_ARGS = [os.environ['VAMPIRE'],
                '--input_syntax', 'tptp', '--proof', 'off']

VAMPIRE_ASSEMBLY_ARGS = ['-sa', 'lrs', '-s', '1010', '-nwc', '10.0'] # was -sa discount...
#VAMPIRE_VALIDATION_ARGS = ['-sa', 'lrs', '-s', '1010', '-nwc', '10.0', '-awr', '1:4']
VAMPIRE_VALIDATION_ARGS = []


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
        # Collect any requirements on the shape of the decision
        requirements = self._get_gap_requirements(gap)

        # Collect possible heuristics with the understanding core
        lines = self._assemble_vampire()
        answer = ''
        for line in lines:
            if 'SZS answers Tuple' in line:
                answer = line
                break
        if answer == '':
            # raise RuntimeError(f'No answer detected in the Vampire output file {file_path}')
            raise RuntimeError('No answer detected in the Vampire output')

        # Select a heuristic that meets the decision requirements
        possible_heuristics = answer.split('|')
        i = 0
        while i < (len(possible_heuristics) - 1):
            heuristic = possible_heuristics[i].split('exert(') #) <-- for editor to find closing brace
            engines = [part[:part.find('_engine')] for part in heuristic if '_engine' in part]
            if self._validate_vampire(engines, requirements):
                return engines

        raise RuntimeError('No valid heuristics created by understanding core')

    def _assemble_vampire(self):
        self.get_logger().info('Running Understanding system')
        VAMPIRE_SHELL = ' '.join(VAMPIRE_ARGS + VAMPIRE_ASSEMBLY_ARGS)
        command = VAMPIRE_SHELL + ' ' + GAP_FILE
        self.get_logger().info('Executing command: ' + command)
        output = subprocess.run(command, shell=True, capture_output=True)

        lines = [line.decode() for line in output.stdout.splitlines()]
        self.get_logger().info('Vampire assembly output:\n' + '\n'.join(lines))
        return lines

    def _validate_vampire(self, engines, requirements):
        self.get_logger().info('Validating heuristic')

        # write validation conjecture to file
        tff_path = os.path.join(self.get_parameter('working_directory').value, 'validate_heuristic.tff')
        with open(tff_path, 'w') as f:
            f.write("include('decision-logic/tff/model/decision-engines.tff').\n")
            f.write("include('decision-logic/tff/model/decision-properties-fixed.tff').\n")
            f.write('tff(question_decl, conjecture,\n')#) <-- for editor to find closing brace
            f.write('  ?[M : modelet')#] <-- for editor to find closing brace
            for i in range(len(engines)):
                f.write(f', MS{i} : modelet_set')
            f.write(']:\n  (\n    (\n') #)) <-- for editor to find closing brace
            for i in range(len(engines) - 1):
                f.write(f'      s(exert({engines[-(1 + i)]}_engine, MS{i}), MS{i}) = MS{i + 1}\n      &\n')
            f.write(f'      exert({engines[0]}_engine, MS{len(engines) - 1}) = M\n')
            f.write('    )\n    =>\n    (\n')
            f.write('      modelet_models_concept(M, decision)\n')
            for requirement in requirements:
                f.write(f'      &\n      modelet_has_property(M, {requirement}_prop)\n')
            f.write('    )\n  )\n).')

        VAMPIRE_SHELL = ' '.join(VAMPIRE_ARGS + VAMPIRE_VALIDATION_ARGS)
        command = VAMPIRE_SHELL + ' ' + tff_path
        self.get_logger().info('Executing command: ' + command)
        output = subprocess.run(command, shell=True, capture_output=True)

        lines = [line.decode() for line in output.stdout.splitlines()]
        self.get_logger().info('Vampire validation output:\n' + '\n'.join(lines))
        
        return 'Refutation'.encode() in output.stdout

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
        self.assertz(f"config_of('{yaml_path}', '{gap}')")

        self.get_logger().info(f'Successfully wrote config to: {yaml_path}')
        self.get_logger().debug(yaml.dump(params))
        return yaml_path

    def write_to_xml(self, gap, pipeline, heuristic_dir, heuristic_name):
        root = ET.Element('root', attrib={'BTCPP_format' : "4"})
        main = ET.SubElement(root, 'BehaviorTree', attrib={'ID': heuristic_name})
        main.append(ET.Element('SubTree', attrib={'ID': f'MakeSureGapClosed_{gap}',
                                                  '_autoremap': 'true', 
                                                  'gap': '{@payload}'}))


        for engine in pipeline:
            meta = os.path.join(get_package_share_directory('heuristic_assembly'), self._get_meta(engine))
            ros_node = self._get_ros_node(engine)
            behavior_tree = ET.parse(meta)
            for node in behavior_tree.iter():
                if node.get('action_name') == '':
                    node.set('action_name', ros_node + '/' + node.tag)
            self._rename_template_subtrees(gap, behavior_tree)
            self._append_all_bts(root, behavior_tree)

        decide_structure = os.path.join(get_package_share_directory('heuristic_assembly'), 
                                        'behavior_trees/decide_structure.xml')
        decide_tree = ET.parse(decide_structure)
        # TODO: remove this hack
        self._rename_template_subtrees(gap, decide_tree)
        self._append_all_bts(root, decide_tree)

        writer = ET.ElementTree(root)
        xml_path = os.path.join(heuristic_dir, f'{gap}.xml')
        writer.write(xml_path)
        self.assertz(f"heuristic_of('{xml_path}', '{gap}')")
        self.assertz(f"entry_point_of('{heuristic_name}', '{gap}')")

        self.get_logger().info(f'Successfully wrote heuristic to: {xml_path}')
        self.get_logger().debug(ET.tostring(root))

        return xml_path

    def _append_all_bts(self, parent, tree):
        parent.extend(tree.iter('BehaviorTree'))

        # Also add includes (assume they are all ROS flavored)
        parent.extend(tree.iter('include'))

    def _rename_template_subtrees(self, gap, tree):
        # TODO: maybe mark templates as unique or not, then only rename the ones
        #       that need to be unique?
        for node in tree.iter():
            node_id = node.get('ID')
            if node_id is not None and node_id.endswith('_Template'):
                node.set('ID', node_id.removesuffix('Template') + gap)

    def _get_gap_requirements(self, gap):
        answers = self.query(f"requirement_of(R, '{gap}')", maxresult=-1) 
        requirements = [answer["R"][:-4] for answer in answers]
        return requirements

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
