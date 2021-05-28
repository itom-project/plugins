import numpy as np

nkx = 4 # number of knots in x
nky = 3 # number of knots in y
nnc = 4 # number of knots per cell
nume = (nkx - 1) * (nky - 1) # total number of elements

KMat = np.zeros([nky, nkx], dtype='uint16')
nk = 0
for ny in range(0, nky):
    for nx in range(0, nkx):
        KMat[ny, nx] = nk
        nk = nk + 1

print(KMat)

for ne in range(0, int(nume)):
    nx = ne % (nkx - 1)
    ny = int(ne / (nkx - 1))
    GMat = np.zeros([2 * nnc, 2 * nkx * nky], dtype='uint8')
    GMat[0, ny * nkx * 2 + nx * 2] = 1
    GMat[1, ny * nkx * 2 + nx * 2 + 1] = 1
    GMat[2, ny * nkx * 2 + (nx + 1) * 2] = 1
    GMat[3, ny * nkx * 2 + (nx + 1) * 2 + 1] = 1
    GMat[4, ny * nkx * 2 + (nx + nkx) * 2] = 1
    GMat[5, ny * nkx * 2 + (nx + nkx) * 2 + 1] = 1
    GMat[6, ny * nkx * 2 + (nx + 1 + nkx) * 2] = 1
    GMat[7, ny * nkx * 2 + (nx + 1 + nkx) * 2 + 1] = 1
    print("ne: ", ne, "nx: ", nx, "ny: ", ny, "\n", GMat)
    #print("Gt G:\n", GMat.transpose().dot(GMat))
#for ny in range(0, nky):
    #for nx in range(0, nky):
        #GMat = np.zeros([2 * nnc, 2 * nkx * nky], dtype='uint8')
        #GMat[ny * 2, nx * 2] = 1
        #GMat[ny * 2 + 1, nx * 2 + 1] = 1
        #GMat[ny * 2 + 2, (nx + 1) * 2] = 1
        #GMat[ny * 2 + 3, (nx + 1) * 2 + 1] = 1
        #GMat[ny * 2 + 4, (nx + nkx) * 2] = 1
        #GMat[ny * 2 + 5, (nx + nkx) * 2 + 1] = 1
        #GMat[ny * 2 + 6, (nx + 1 + nkx) * 2] = 1
        #GMat[ny * 2 + 7, (nx + 1 + nkx) * 2 + 1] = 1
        #print(GMat)