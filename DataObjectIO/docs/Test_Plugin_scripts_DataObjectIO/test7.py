a=dataObject([100,100],'float32')
a[0,0]=23454
for i in range(0,100):
    for j in range(0,100):
        a[i,j]=45223+(i+j)*20
filter("saveDataObject",a,"E:\\shishir\\test_ARGB32_scaled.jpg","QImage::Format_ARGB32")
