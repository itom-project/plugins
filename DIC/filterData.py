import csv
import numpy as np

filenameLoad = "D:\\data\\ufal\\DIC\\Statistics\\anovaCSV_7.csv"
filenameSave = "D:\\data\\ufal\\DIC\\Statistics\\anovaCSV_7_2.csv"

csvfile = open(filenameLoad, newline='')
csvreader = csv.reader(csvfile)
csvreader.__next__()
csvreader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)

data=list(csvreader)
data=np.array(data)
del csvreader
del csvfile

plot(data[:,1])
plot(data[:,5])

data2=data[data[:,3] != 5]

apprMean0 = np.mean(data2[data2[:,0] == 0][:,5])
apprVar0 = np.std(data2[data2[:,0] == 0][:,5])
apprCnt0 = len(data2[data2[:,0] == 0][:,5])
apprMean1 = np.mean(data2[data2[:,0] == 1][:,5])
apprVar1 = np.std(data2[data2[:,0] == 1][:,5])
apprCnt1 = len(data2[data2[:,0] == 1][:,5])

apprMean0 = np.mean(data2[data2[:,0] == 0][:,5])
apprVar0 = np.std(data2[data2[:,0] == 0][:,5])
apprCnt0 = len(data2[data2[:,0] == 0][:,5])
apprMean1 = np.mean(data2[data2[:,0] == 1][:,5])
apprVar1 = np.std(data2[data2[:,0] == 1][:,5])
apprCnt1 = len(data2[data2[:,0] == 1][:,5])

OptMean0 = np.mean(data2[data2[:,1] == 0][:,5])
OptVar0 = np.std(data2[data2[:,1] == 0][:,5])
OptCnt0 = len(data2[data2[:,1] == 0][:,5])
OptMean1 = np.mean(data2[data2[:,1] == 1][:,5])
OptVar1 = np.std(data2[data2[:,1] == 1][:,5])
OptCnt1 = len(data2[data2[:,1] == 1][:,5])

NormMean0 = np.mean(data2[data2[:,4] == 0][:,5])
NormVar0 = np.std(data2[data2[:,4] == 0][:,5])
NormCnt0 = len(data2[data2[:,4] == 0][:,5])
NormMean1 = np.mean(data2[data2[:,4] == 1][:,5])
NormVar1 = np.std(data2[data2[:,4] == 1][:,5])
NormCnt1 = len(data2[data2[:,4] == 1][:,5])

CellMean9 = np.mean(data2[data2[:,3] == 9][:,5])
CellVar9 = np.std(data2[data2[:,3] == 9][:,5])
CellCnt9 = len(data2[data2[:,3] == 9][:,5])
CellMean33 = np.mean(data2[data2[:,3] == 33][:,5])
CellVar33 = np.std(data2[data2[:,3] == 33][:,5])
CellCnt33 = len(data2[data2[:,3] == 33][:,5])
CellMean65 = np.mean(data2[data2[:,3] == 65][:,5])
CellVar65 = np.std(data2[data2[:,3] == 65][:,5])
CellCnt65 = len(data2[data2[:,3] == 65][:,5])

IntMean2 = np.mean(data2[data2[:,2] == 10][:,5])
IntVar2 = np.std(data2[data2[:,2] == 10][:,5])
IntCnt2 = len(data2[data2[:,2] == 10][:,5])
IntMean3 = np.mean(data2[data2[:,2] == 3][:,5])
IntVar3 = np.std(data2[data2[:,2] == 3][:,5])
IntCnt3 = len(data2[data2[:,2] == 3][:,5])
IntMean5 = np.mean(data2[data2[:,2] == 5][:,5])
IntVar5 = np.std(data2[data2[:,2] == 5][:,5])
IntCnt5 = len(data2[data2[:,2] == 5][:,5])
IntMean7 = np.mean(data2[data2[:,2] == 7][:,5])
IntVar7 = np.std(data2[data2[:,2] == 7][:,5])
IntCnt7 = len(data2[data2[:,2] == 7][:,5])

print("        Approach Optimization Normalization CellSize Interpolation")
print(f"Cnt 0 : {apprCnt0}      {OptCnt0}          {NormCnt0}           {CellCnt9}      {IntCnt2}")
print(f"Mean 0: {apprMean0}      {OptMean0}          {NormMean0}           {CellMean9}      {IntMean2}")
print(f"Sdev 0: {apprVar0}      {OptVar0}          {NormVar0}           {CellVar9}      {IntVar2}")
print("")
print(f"Cnt 1 : {apprCnt1}      {OptCnt1}          {NormCnt1}           {CellCnt33}      {IntCnt3}")
print(f"Mean 1: {apprMean1}      {OptMean1}          {NormMean1}           {CellMean33}      {IntMean3}")
print(f"Sdev 1: {apprVar1}      {OptVar1}          {NormVar1}           {CellVar33}      {IntVar3}")
print("")
print(f"Cnt 2 :                                     {CellCnt65}      {IntCnt5}")
print(f"Mean 2:                                     {CellMean65}      {IntMean5}")
print(f"Sdev 2:                                     {CellVar65}      {IntVar5}")
print("")
print(f"Cnt 3 :                                              {IntCnt7}")
print(f"Mean 3:                                              {IntMean7}")
print(f"Sdev 3:                                              {IntVar7}")

csvfile = open(filenameSave, newline='', mode='w')
csvwriter = csv.writer(csvfile)
csvwriter.writerow(['Appr', 'Optimization', 'Int', 'Cell', 'Norm', 'Error'])
csvwriter.writerows(data2)
del csvwriter
del csvfile
