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

    :param assessments: A numpy array of assessment scores of shape (n_alternatives x n_cues).
    :param weights: A dicitonary with cue names as keys, and float weights as values.
    :param normalize: If True, linearly transform each assessment over its to
        the range `[0,1]`. Defaults to False

    :return: A list of utility scores for each assessed alternative.
    """

    if normalize:
        mins = np.min(assessments, axis=0)
        maxs = np.max(assessments, axis=0)
        assessments = (assessments - mins) / (maxs - mins)

    # Assemble weight vector in same order
    weight_vector = np.zeros(len(assessments))
    for i, assessment in enumerate(assessments):
        weight_vector[i] = weights[assessment.cue.id]

    # calculate utilities
    utilities = assessments @ weight_vector
    return utilities.tolist()


def boolean_combination(assessments, operator='and'):
    """Combine assessments via a boolean operation.

    :param assessments: A numpy array of assessment scores of shape (n_alternatives x n_cues).
    :param operator: Combine assessments via this logical operator. Valid
        options are `'or'` or `'and'`, defaults to 'and'

    :raises ValueError: If the operator is invalid.

    :return: A list of utility scores for each assessed alternative.
    """
    match operator.lower():
        case 'and':
            truthiness = np.all(assessments.astype(bool), axis=1)
        case 'or':
            truthiness = np.any(assessments.astype(bool), axis=1)
        case _:
            raise ValueError(f"Recieved invalid logical operator: '{operator}'.")

    return truthiness.astype(float).tolist()


def tallying(assessments):
    """Combine assessments by tallying positive scores. Assume 0 is negative!

    :param assessments: A numpy array of assessment scores of shape (n_alternatives x n_cues).

    :return: A list of utility scores for each assessed alternative.
    """
    tallies = np.sum(assessments > 0, axis=1)
    return tallies.astype(float).tolist()


def dawes_rule(assessments):
    """Combine assessments via Dawes' rule. Assume 0 is negative!

    :param assessments: A numpy array of assessment scores of shape (n_alternatives x n_cues).

    :return: A list of utility scores for each assessed alternative.
    """
    dawes = np.sum(assessments > 0, axis=1) - np.sum(assessments <= 0, axis=1)
    return dawes.astype(float).tolist()


