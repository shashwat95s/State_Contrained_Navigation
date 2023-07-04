#! /usr/bin/env python3

def range_smoother(range_info,min,max):
    smooth_array = []
    for i in range(len(range_info)):
        if(range_info[i]==min):
            continue
        if(range_info[i]==max):
            continue
        smooth_array.append(range_info[i])
    return smooth_array