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


def weighted_sum(assessments, weights, normalize=False):
    """Return a weighted sum of each assessment.

    :param assessments: A list of :class:`decision_msg.msg.Assessment` assessments.
    :param weights: A dicitonary with cue names as keys, and float weights as values.
    :param normalize: If True, linearly transform each assessment over its to
        the range `[0,1]`. Defaults to False

    :return: A list of utility scores for each assessed alternative.
    """
    assessment_matrix = create_assessment_matrix(assessments) 

    if normalize:
        mins = np.min(assessment_matrix, axis=0)
        maxs = np.max(assessment_matrix, axis=0)
        assessment_matrix = (assessment_matrix - mins) / (maxs - mins)

    # Assemble weight vector in same order
    weight_vector = np.zeros(len(assessments))
    for i, assessment in enumerate(assessments):
        weight_vector[i] = weights[assessment.cue.id]

    # calculate utilities
    utilities = assessment_matrix @ weight_vector
    return utilities.tolist()


def create_assessment_matrix(assessments):
    """Create a matrix of features from a set of assessments

    :param assessments: A list of :class:`decision_msg.msg.Assessment` assessments.

    :return: A numpy array of assessment scores of shape (n_alternatives x n_cues).
    """
    # convert assessments into an A x C matix
    n_alternatives = len(assessments[0].preferences)
    n_assessments = len(assessments)
    assessment_matrix = np.zeros((n_alternatives, n_assessments))
    
    for i, assessment in enumerate(assessments):
        # Assume all assessments have the same alternatives in the same order
        assessment_matrix[:,i] = [p.score for p in assessment.preferences]

    return assessment_matrix


def boolean_combination(assessments, operator='and'):
    """Combine assessments via a boolean operation.

    :param assessments: A list of :class:`decision_msg.msg.Assessment` assessments.
    :param operator: Combine assessments via this logical operator. Valid
        options are `'or'` or `'and'`, defaults to 'and'

    :raises ValueError: If the operator is invalid.

    :return: A list of utility scores for each assessed alternative.
    """
    assessment_matrix = create_assessment_matrix(assessments) 

    match operator.lower():
        case 'and':
            truthiness = np.all(assessment_matrix.astype(bool), axis=1)
        case 'or':
            truthiness = np.any(assessment_matrix.astype(bool), axis=1)
        case _:
            raise ValueError(f"Recieved invalid logical operator: '{operator}'.")

    return truthiness.astype(float).tolist()


def tallying(assessments):
    """Combine assessments by tallying positive scores. Assume 0 is negative!

    :param assessments: A list of :class:`decision_msg.msg.Assessment` assessments.

    :return: A list of utility scores for each assessed alternative.
    """
    assessment_matrix = create_assessment_matrix(assessments) 

    tallies = np.sum(assessment_matrix > 0, axis=1)
    return tallies.astype(float).tolist()


def dawes_rule(assessments):
    """Combine assessments via Dawes' rule. Assume 0 is negative!

    :param assessments: A list of :class:`decision_msg.msg.Assessment` assessments.

    :return: A list of utility scores for each assessed alternative.
    """
    assessment_matrix = create_assessment_matrix(assessments) 

    dawes = np.sum(assessment_matrix > 0, axis=1) - np.sum(assessment_matrix <= 0, axis=1)
    return dawes.astype(float).tolist()


