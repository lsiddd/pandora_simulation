#!/usr/bin/env python2.7
import warnings

def fxn():
    warnings.warn("deprecated", DeprecationWarning)

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    fxn()

import sys
import numpy as np
#i = 0.6
#p = b = 0.7
#g = 12

I = sys.argv[1]
P = sys.argv[2]
B = sys.argv[3]
G = sys.argv[4]

def calc (i, p, b, g):

    vec = [i, p, b, g]
    vec = np.array(vec).reshape(1,-1)

    import six.moves.cPickle as pickle
    import cPickle as pickle
    arquivo_pkl = open('ia_etr.pkl', 'r')
    #arquivo_pkl = open(r '/home/bittrooper/ia_etr.pkl')
    clf = pickle.load(arquivo_pkl)
    m = clf.predict(vec)[0]
    return m
print (calc(I, P, B, G))
