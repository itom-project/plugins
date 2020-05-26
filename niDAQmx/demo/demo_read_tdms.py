# coding=utf8

"""Demo to load and read a TDMS file

Here, we read the TMDS files, that have been created
by the ai_continuous and di_continuous demo scripts.

This script requires the Python package npTDMS
(https://pypi.org/project/npTDMS).
"""

import nptdms as tdms
import numpy as np

# step 1: read the file demo_ai_continuous.tdms

file = tdms.TdmsFile(r"D:\temp\demo_ai_continuous.tdms")

print("Available groups:")
print(file.groups())

# access group object
groupObject= file.object(file.groups()[0])


data = []

for obj in file.group_channels('group1'):
    data.append(obj.data)

total2= np.vstack(data)
plot1(total2)
