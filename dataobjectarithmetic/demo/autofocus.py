import time
a = dataObject.randN([7, 100,120])
a[:,0:50,:] *= 0.5
a[:, 50:100,:] *= 1.5
b = dataObject()
filter("lowPassFilter",a[3,:,:], b, 7, 7)
a[0,:,:] = b
a[6,:,:] = b
filter("lowPassFilter",a[3,:,:], b, 5, 5)
a[1,:,:] = b
a[5,:,:] = b
filter("lowPassFilter",a[3,:,:], b, 3, 3)
a[2,:,:] = b
a[4,:,:] = b


meth = ["3x3Diff", "3x3Sobel", "3x3Scharr", "3x3Roberts", "3x3Prewitt", "5x5Sobel", "Gradient", "3x3Laplacian", "5x5Laplacian"]
estimates = dataObject([len(meth), 7], 'float64')
i = 0
for m in meth:
    t = time.time()
    result = filter("autofocus",a, m)
    print(m, result, time.time()-t)
    minimum = min(result)
    estimates[i,:] = [r / minimum for r in result] #normalize to minimum
    i+=1
plot(estimates, '1d', properties = {"columnInterpretation":"MultiRows", "legendPosition":"Right", "legendTitles":meth})
