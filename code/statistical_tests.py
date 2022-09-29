""" In this file various statistical tests are performed in order to analyse the performance of the various methods employed"""

import numpy as np
import scipy.stats as st

def averageVariance(set):
    """
    Computes the average and the variance of a data sets, then it returns it
    """
    avg = np.mean(set)
    var = np.var(set)

    return avg, var


def simulatesUntilNormalStop(l, alpha = 0.05):
    """ simulates, then decides when to stop generating data for Normal distribution
    Args:
        l (float) - length of the confidence interval
        alpha (float) - significance level
    """
    z_alpha2 = st.norm.pdf(1 - alpha/2)
    size = 1 # REMOVE
    # generate 100 samples
    k = 1

    while (2 * z_alpha2 * size / np.sqrt(k) < l):
        k = +1

        