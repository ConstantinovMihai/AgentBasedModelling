""" In this file various statistical tests are performed in order to analyse the performance of the various methods employed"""

import numpy as np
import scipy.stats as stats
 

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
    z_alpha2 = stats.norm.pdf(1 - alpha/2)
    size = 1 # REMOVE
    # generate 100 samples
    k = 1

    while (2 * z_alpha2 * size / np.sqrt(k) < l):
        k = +1


def shapiroWilk(data : list, significance_lvl : float = 0.05):
    """ performs the Shapiro Wilk test and checks for normality for a given realization

    Args:
        data (list): data to be checked for normality
    
    Returns a bool, concerning whether the data came from a normal distribution (True) or not (False), depending on the sifnificance lvl
    """

    shapiro_test = stats.shapiro(data)
    return shapiro_test.pvalue > significance_lvl


if __name__ == "__main__":
    rng = np.random.default_rng()
    x = stats.norm.rvs(loc=5, scale=3, size=100, random_state=rng.binomial)
    print(f'is data normal? {shapiroWilk(x)}')

    