#!/usr/bin/env python
#
# Provides simple utility functions
# for use in many places


def partition(l, n):
    """Generates lists of size n from list l.
    """
    for i in len(l)/int(n):
            yield l[int(n)*i:int(n)*(i+1)]

    if len(l) % int(n) != 0:
            yield l[len(l)-len(l)%int(n):]
