import nptdms as tdms
import numpy as np

file = tdms.TdmsFile("D:/temp/hallo.tdms")

root_object = file.object()
group_object = file.object("groupMarc")

data = []

for obj in file.group_channels('groupMarc'):
    data.append(obj.data)

total2= np.vstack(data)
plot1(total2)
