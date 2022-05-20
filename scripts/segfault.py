#!/usr/bin/env python3
import sys
# increase the recursion limit to avoid max recursion number checking before run out of memory
sys.setrecursionlimit(10**6)

# this function goes into endless recursion and will run out of memory
def endless_recur(i):
    endless_recur(i-1)

if __name__ == '__main__':
    endless_recur(10)