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


def validate_matrix(rows, cols, items):
    """Assert that the recieved sizes of matrix data are valid.

    :param rows: The number of row labels.
    :param cols: The number of column labels.
    :param items: The number of items in the matrix.

    :raises ValueError: if either dimension is empty or the total number of items
        is not the multiple of the number of rows and columns.

    """
    if cols < 1:
        raise ValueError('Recieved empty list of column headers')
    if rows < 1:
        raise ValueError('Recieved empty list of row headers')
    if cols * rows != items:
        raise ValueError('Incompatible matrix. '
                         f'Expected shape ({rows} x {cols})'
                         f' = {cols * rows} items), but got {items} items.')


def scores_to_np_array(scores, n_alternatives):
    return np.array(scores).reshape((n_alternatives, -1))
