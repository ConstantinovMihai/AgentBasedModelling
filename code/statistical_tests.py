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
        significance_lvl (float, optional): the significance level under which the Null hypothesis is rejected. Defaults to 0.05.
    
    Returns a bool, concerning whether the data came from a normal distribution (True) or not (False), depending on the sifnificance lvl
    """

    shapiro_test = stats.shapiro(data)
    
    # if True, the null hypothesis is accepted (data is drawn from a normal distribution)
    return shapiro_test.pvalue > significance_lvl


def mannWhitneyUTest(data1 : list, data2 : list, significance_lvl : float = 0.05):
    """ Performs the Mann Whitney U test on two sets of sampled data (not normally distributed according to the Shapiro Wilk test)

    Args:
        data1 (list): data sampled from the first distribution
        data2 (list): data sampled from the second distribution
        significance_lvl (float, optional): the significance level under which the Null hypothesis is rejected. Defaults to 0.05.
    
    Returns a bool concerning whether or not the Null hypothesis is rejected
    """

    mannwhitneyu_test = stats.mannwhitneyu(data1, data2, method="exact")
    print(f"mannwhitneyu_test.pvalue {mannwhitneyu_test.pvalue}, p_value {significance_lvl}")
    # if True, the alternative hypothesis is accepted (the true difference of the 2 distributions is indeed not 0)
    return mannwhitneyu_test.pvalue < significance_lvl
    


def unpairedTTest(data1 : list, data2 : list, significance_lvl : float = 0.05):
    """ Performs the unpaired t test on two sets of sampled data (normally distributed according to the Shapiro Wilk test)

    Args:
        data1 (list): data sampled from the first distribution
        data2 (list): data sampled from the second distribution
        significance_lvl (float, optional): the significance level under which the Null hypothesis is rejected. Defaults to 0.05.
    
    Returns a bool concerning whether or not the Null hypothesis is rejected
    """
    
    ttest_unpaired = stats.ttest_ind(data1, data2)

    # if True, the alternative hypothesis is accepted (the true difference of the 2 distributions is indeed not 0)
    return ttest_unpaired.pvalue < significance_lvl



def statisticalTests(data1 : list, data2 : list, significance_lvl : float = 0.05):
    """ Performs statistical tests on sample data coming from two unpaired distribution

    Args:
        data1 (list): data sampled from the first distribution
        data2 (list): data sampled from the second distribution
        significance_lvl (float, optional): the significance level under which the Null hypothesis is rejected. Defaults to 0.05.
    
    Returns a bool concerning whether or not the Alternative hypothesis is accepted, i.e. if True - different means
    """

    # if both samples come from normally distributed data, then run the unpaired t test, otherwise use Mann Whitney U test
    if shapiroWilk(data1, significance_lvl) and shapiroWilk(data2, significance_lvl):
        return unpairedTTest(data1, data2, significance_lvl)

    return  mannWhitneyUTest(data1, data2, significance_lvl)

if __name__ == "__main__":
    rng = np.random.default_rng()
    x = stats.norm.rvs(loc=5, scale=3, size=100, random_state=rng)
    print(f'is data normal? {shapiroWilk(x)}')

    