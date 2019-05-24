#!/usr/bin/env python
"""
This files attempts to read the description.cvxgen file and generates the
files necessary to generate an python handle using SWIG.
"""

import sys

def get_next_end(index, ends):
    for e in ends:
        if e > index:
            return e
    return 0


lines = [] #Declare an empty list named "lines"
with open ('cvxgen/description.cvxgen', 'rt') as description_file:  #Open file lorem.txt for reading of text data.
    for line in description_file:
        lines.append(line)

ends = []
for (index, line) in enumerate(lines):
    if "dimensions\n" in line:
        dimension = index+1
    if "parameters\n" in line:
        parameter = index+1
    if "variables\n" in line:
        variable = index+1
    if "end\n" in line:
        ends.append(index)

### Handle the dimensions - Read to a dictionary and ignore any comments
dimension_dict = {}
dimension_end = get_next_end(dimension, ends)
for line in lines[dimension:dimension_end]:
    line_clean = line.replace(' ','')
    exec(line_clean)
    key = line_clean.split("=")[0]
    dimension_dict[key]= eval(key)
print dimension_dict

### Handle the dimensions - Read to a dictionary and ignore any comments
dimension_dict = {}
dimension_end = get_next_end(dimension, ends)
for line in lines[dimension:dimension_end]:
    line_clean = line.replace(' ','')
    exec(line_clean)
    key = line_clean.split("=")[0]
    dimension_dict[key]= eval(key)
print dimension_dict
