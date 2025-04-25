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

from operator import itemgetter
import random


def create_ordered_pairs(alternatives, ranks):
    """Create a list of pairs of ``alternatives`` and their ``ranks`` ordered by rank.

    :param alternatives: A list of :class`decision_interfaces.msg.Alternative` 
        alternatives under considereation.
    :param ranks: A list of positive integer ranks, corresponding to each
        alternative.

    :raises ValueError: If the list of ``alternatives`` is empty or ``ranks`` and
        ``alternatives`` have unequal lengths.

    :return: A list of a list of pairs of ``alternatives`` and their ``ranks``
        ordered by rank.
    """
    n_alternatives = len(alternatives)
    n_ranks = len(ranks)

    if n_alternatives < 1:
        raise ValueError('Recieved empty list of alternatives')
    if n_alternatives != n_ranks:
        raise ValueError(f"Ordering contains lists of unequal lengths: alternatives={n_alternatives} ranks={n_ranks}")

    return sorted(zip(alternatives, ranks), key = itemgetter(1))


def _choose_with_ties(ranked_alternatives, start):
    i = _march_backward(ranked_alternatives, start)
    j = _march_forward(ranked_alternatives, start)

    choice = []
    if i > 0:
        best, _ = zip(*ranked_alternatives[:i])
        choice += best
    if j - i > 0:
        ties, _ = zip(*random.sample(ranked_alternatives[i:j], k=start-i+1))
        choice += ties

    return choice


def _march_forward(ranked_alternatives, start):
    i = start
    tied_rank = ranked_alternatives[start][1]
    while i < len(ranked_alternatives) and ranked_alternatives[i][1] == tied_rank:
        i += 1
    return i


def _march_backward(ranked_alternatives, start):
    i = start
    tied_rank = ranked_alternatives[start][1]
    while i > 0 and ranked_alternatives[i-1][1] == tied_rank:
        i -= 1
    return i


def take_best(ranked_alternatives, n=None, random_ties=False):
    """Takes the best alternatives.

    :param ranked_alternatives: A list of pairs of :class:`decision_interfaces.msg.Alternative`
        alternatives and integer ranks sorted by rank.
        under consideration.
    :param n: The number of best alternatives to return. Assume all are taken
        if this is larger than the number of ``alternatives``.
    :type n: int, optional
    :param random_ties: If true, randomly select between tied alternatives, default to False.
    :type random_ties: boolean, optional

    :return: A list of chosen alternatives.

    """
    if n is None:
        chosen, _ = zip(*ranked_alternatives[:_march_forward(ranked_alternatives, 0)])
    elif random_ties:
        return _choose_with_ties(ranked_alternatives, n - 1)
    else:
        chosen, _ = zip(*ranked_alternatives[:n])
    return chosen


def eliminate_worst(ranked_alternatives, n=None, random_ties=False):
    """Eliminates the worst alternatives.

    :param ranked_alternatives: A list of pairs of :class:`decision_interfaces.msg.Alternative`
        alternatives and integer ranks sorted by rank.
        under consideration.
    :param n: The number of worst alternatives to remove.  Assume none are taken
        if this is larger than the number of ``alternatives``.
    :type n: int, optional
    :param random_ties: If true, randomly select between tied alternatives, default to False.
    :type random_ties: boolean, optional

    :return: A list of chosen alternatives.

    """
    if n is None: # Eliminate all tied for worst score
        chosen, _ = zip(*ranked_alternatives[:_march_backward(ranked_alternatives, len(ranked_alternatives) - 1)])
    elif random_ties:
        return _choose_with_ties(ranked_alternatives, len(ranked_alternatives) - n - 1)
    else:
        chosen, _ = zip(*ranked_alternatives[:-n])
    return chosen
