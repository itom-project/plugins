# -*- coding: utf-8 -*-
#"""
#Created on Tue Nov 29 20:07:23 2011
#
#@author: chris
#"""
import matplotlib
matplotlib.use('module://mpl_itom.backend_itomagg')
import pylab as py

imgmat = py.zeros([100,100])
imgmat[40:60, 40:60] = 1
imgmat10 = imgmat * 10
noisemin = -10
noisemax = 10

imgmat10avg = py.zeros([100,100])
for i in range(0,20):
    imgmat10n = imgmat10 + py.randint(noisemin,noisemax,[100,100])
    imgmat10avg = imgmat10avg + imgmat10n / 20

py.figure()
py.imshow(imgmat10n, interpolation='nearest')
py.show()

py.figure()
py.imshow(imgmat10avg, interpolation='nearest')
py.show()
