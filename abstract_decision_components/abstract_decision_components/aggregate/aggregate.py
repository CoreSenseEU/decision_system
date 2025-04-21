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

import numpy as np


def weighted_sum(assessments, weights):
    """Return a weighted sum of each assessment.
    """
    assessment_matrix = create_assessment_matrix(assessments) 

    # Assemble weight vector in same order
    weight_vector = np.zeros((len(assessments),1))
    for i, assessment in enumerate(assessments):
        weight_vector[i] = weights[assessment.cue.id]

    # calculate utilities
    utilities = assessment_matrix @ weight_vector
    return utilities.to_list()


def create_assessment_matrix(assessments):
    """Create a matrix of features from a set of judgments
    """
    # convert assessments into an A x C matix
    n_alternatives = len(assessments[0].preferences)
    n_assessments = len(assessments)
    assessment_matrix = np.zeros((n_alternatives, n_assessments))
    
    for i, assessment in enumerate(assessments):
        # Assume all judgments have the same features in the same order
        assessment_matrix[:,i] = [p.score for p in assessment.preferences]

    return assessment_matrix


def boolean_combination(assessments, operator):
    """Combine assessments via a boolean operation.
    """
    assessment_matrix = create_assessment_matrix(assessments) 

    match operator.lower():
        case 'and':
            truthiness = np.all(assessment_matrix.astype(bool), axis=1)
        case 'or':
            truthiness = np.any(assessment_matrix.astype(bool), axis=1)
        case _:
            raise ValueError(f"Recieved invalid logical operator: '{operator}'.")

    return truthiness.astype(float).to_list()


def dawes_rule(assessments):
    """Combine assessments via Dawes' rule. Assume 0 is negative!
    """
    assessment_matrix = create_assessment_matrix(assessments) 

    dawes = np.sum(assessment_matrix > 0, axis=1) - np.sum(assessment_matrix <= 0, axis=1)
    return dawes

