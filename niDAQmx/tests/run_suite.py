import unittest
from setParams_error_reporting import setParam
from getParams_error_reporting import getParam
from misc_error_reporting import miscTests
from Peripheral_error_reporting import PeripheralTests
from Read_channel_finite import ReadChannelFinite

if (__name__ == "__main__"):

    # run test suite

    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(setParam))
    suite.addTest(unittest.makeSuite(getParam))
    suite.addTest(unittest.makeSuite(miscTests))
    suite.addTest(unittest.makeSuite(PeripheralTests))
    suite.addTest(unittest.makeSuite(ReadChannelFinite))
    unittest.TextTestRunner(verbosity=2).run(suite)
