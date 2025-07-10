"""
Postprocessing algorithms for when all predictions are done when ended execution of robot.

This file implements adjustation to cotton states, for states that can be confusing or misleading.

    - Cotton sorting by distance to given cotton c::Cotton
    - KNN algorithm given radius r::float, labels sorted
    - Apply Markov chains to get which path to follow from possible outcomes. Possible results of algorithm are:
        · Most probable state with a u->v path in Markov matrix
        · Shortest path in Markov, |u->v| > 1, with given result possible in KNN neighbours with determined quantile q

    Variables of problem are:

    - World state of cotton predictions P
    - Fuzzy radius r
    - Quantile of acceptance q (given world state, the centered quantile q for which predictions to take in acount)
    - Cotton to be corrected c

    For every cotton predicted c, relevant information to take into account is:

    - Coordinates x,y,z
    - Category of label l 
"""

# Generic modules
import numpy as np

# Own modules
from ..utils.cotton import Cotton

class Postprocessing(object):
    """
    Implements Markov matrices, KNN neighbours and decision making for fuzzy states in prediction.
    """

    def __init__(self) -> None:

        self.M = np.array([])       # Markov matrix TODO
        pass

    def knn(self, cot: Cotton, worldstate: "WorldState"):
        """
        TODO
        """

        pass

    def markov(self, cot: Cotton):
        """
        Apply markov matrices 
        """
        pass

