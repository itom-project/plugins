import unittest
from base_data import param_strings
from itom import dataIO

class getParam(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.plugin=dataIO("niDAQmx")

    @classmethod
    def tearDownClass(self):
        del self.plugin
    
    def test_calling_ciChParams_not_yet_implemented(self):
        strings = param_strings
        with self.assertRaises(RuntimeError) as context:
            self.plugin.getParam("ciChParams")
        self.assertTrue('niDAQmx::getParam - ciChParams. Counter input is not implemented' in str(context.exception))

    def test_calling_coChParams_not_yet_implemented(self):
        strings = param_strings
        with self.assertRaises(RuntimeError) as context:
            self.plugin.getParam("coChParams")
        self.assertTrue('niDAQmx::getParam - coChParams. Counter output is not implemented' in str(context.exception))

    def test_calling_aoTaskParams_not_yet_implemented(self):
        strings = param_strings
        with self.assertRaises(RuntimeError) as context:
            self.plugin.getParam("aoTaskParams")
        self.assertTrue('niDAQmx::getParam - aoTaskParams. aoTaskParams is not implemented' in str(context.exception))

    def test_calling_diTaskParams_not_yet_implemented(self):
        strings = param_strings
        with self.assertRaises(RuntimeError) as context:
            self.plugin.getParam("diTaskParams")
        self.assertTrue('niDAQmx::getParam - diTaskParams. diTaskParams is not implemented' in str(context.exception))

    def test_calling_doTaskParams_not_yet_implemented(self):
        strings = param_strings
        with self.assertRaises(RuntimeError) as context:
            self.plugin.getParam("doTaskParams")
        self.assertTrue('niDAQmx::getParam - doTaskParams. doTaskParams is not implemented' in str(context.exception))

    def test_calling_ciTaskParams_not_yet_implemented(self):
        strings = param_strings
        with self.assertRaises(RuntimeError) as context:
            self.plugin.getParam("ciTaskParams")
        self.assertTrue('niDAQmx::getParam - ciTaskParams. ciTaskParams is not implemented' in str(context.exception))

    def test_calling_coTaskParams_not_yet_implemented(self):
        strings = param_strings
        with self.assertRaises(RuntimeError) as context:
            self.plugin.getParam("coTaskParams")
        self.assertTrue('niDAQmx::getParam - coTaskParams. coTaskParams is not implemented' in str(context.exception))


if __name__ == '__main__':
    unittest.main()
