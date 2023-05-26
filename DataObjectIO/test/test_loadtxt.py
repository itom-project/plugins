excel_export = dataObject()
rhino_export = dataObject()

import unittest
from itom import filter, dataObject
import numpy as np

class LoadTxtTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    def test_excelExport(self):
        d = dataObject()
        filter("loadTXT", d, "excel_export.csv")
        expect = dataObject([2,2],'float32', data=(1.4, 5, 2.3, 4))
        cmp = (d == expect)
        self.assertEqual(np.sum(cmp), 4*255)

    def test_rhinoExport(self):
        d = dataObject()
        filter("loadTXT", d, "rhino_export.txt", wrapSign = "\"")
        self.assertEqual(d.ndim, 2)
        self.assertEqual(d.shape[0], 4)
        self.assertEqual(d.shape[1], 3)
        self.assertLess(np.abs(d[0,0]+9.5999756), 0.01)

    def test_germanstyle(self):
        d = dataObject()
        filter("loadTXT", d, "germanstyle.txt", separatorSign=";", decimalSign=",")
        expect = dataObject([1,2],'float32', data=(20000, 30000.1))
        cmp = (d == expect)
        self.assertEqual(np.sum(cmp), 2*255)

    def test_englishstyle(self):
        d = dataObject()
        filter("loadTXT", d, "englishstyle.txt", separatorSign=";", decimalSign=".")
        expect = dataObject([1,2],'float32', data=(20000, 30000.1))
        cmp = (d == expect)
        self.assertEqual(np.sum(cmp), 2*255)

    def test_chaosstyle(self):
        d = dataObject()
        filter("loadTXT", d, "chaosstyle.txt", ignoreLines = 1, separatorSign=";", decimalSign=".")
        expect = dataObject([2,2],'float32', data=(20000, 30000.1, 30.12, float('nan')))
        cmp = (d == expect)
        self.assertEqual(np.sum(cmp), 3*255)



if __name__ == '__main__':
    unittest.main()
