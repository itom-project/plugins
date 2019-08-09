import unittest
from base_data import param_strings
from itom import dataIO

class setParam(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.plugin=dataIO("niDAQmx")

    @classmethod
    def tearDownClass(self):
        del self.plugin
    
    def test_calling_aiChParams_before_aiTaskParams_is_called(self):
        strings = param_strings
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + "-10" + "10"
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiChParams", aiChParams)
        self.assertTrue('niDAQmx::setParam - aiChParams. Your device does not support this channel or the task is not initialized' in str(context.exception))

    def test_calling_aoChParams_before_aoTaskParams_is_called(self):
        strings = param_strings
        aoChParams = strings["aoChParamsDev"] + strings["aoChParamsCh"] + "," + strings["aoChParamsMinOutputLim"] + "," + strings["aoChParamsMaxOutputLim"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aoChParams", aoChParams)
        self.assertTrue('niDAQmx::setParam - aoChParams. Your device does not support this channel or the task is not initialized' in str(context.exception))

    def test_calling_diChParams_before_diTaskParams_is_called(self):
        strings = param_strings
        diChParams = strings["diChParamsDev"] + strings["diChParamsCh"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("diChParams", diChParams)
        self.assertTrue('niDAQmx::setParam - diChParams. Your device does not support this port or the task is not initialized' in str(context.exception))

    def test_calling_doChParams_before_doTaskParams_is_called(self):
        strings = param_strings
        doChParams = strings["doChParamsDev"] + strings["doChParamsCh"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("doChParams", doChParams)
        self.assertTrue('niDAQmx::setParam - doChParams. Your device does not support this port or the task is not initialized' in str(context.exception))

    def test_calling_ciChParams_before_ciTaskParams_is_called(self):
        strings = param_strings
        ciChParams = strings["ciChParamsDev"] + strings["ciChParamsCh"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("ciChParams", ciChParams)
        self.assertTrue('niDAQmx::setParam - ciChParams. Counter input is not implemented' in str(context.exception))

    def test_calling_coChParams_before_coTaskParams_is_called(self):
        strings = param_strings
        coChParams = strings["coChParamsDev"] + strings["coChParamsCh"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("coChParams", coChParams)
        self.assertTrue('niDAQmx::setParam - coChParams. Counter output is not implemented' in str(context.exception))

    def test_calling_aiTaskParams_with_not_enough_parameters(self):
        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.assertTrue('niDAQmx::setParam - TaskParams. Wrong number of parameters (rate[Hz], samples[1], mode)' in str(context.exception))

    def test_calling_aiTaskParams_with_invalid_TaskParams(self):
        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + "3"
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.assertTrue('niDAQmx::setParam - TaskParams. Task Parmaters mode invalid' in str(context.exception))

    def test_calling_aiTaskParams_with_mode_on_demand(self):
        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMOnDemand"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.assertTrue('niDAQmx::setParam - TaskParams. On demand mode is not yet supported' in str(context.exception))

    def test_calling_aoTaskParams_not_yet_implemented(self):
        strings = param_strings
        aoTaskParams = strings["aoTaskParamsSR"] + strings["aoTaskParamsS"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aoTaskParams", aoTaskParams)
        self.assertTrue('niDAQmx::setParam - aoTaskParams. aoTaskParams is not implemented' in str(context.exception))

    def test_calling_diTaskParams_not_yet_implemented(self):
        strings = param_strings
        diTaskParams = strings["diTaskParamsSR"] + strings["diTaskParamsS"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("diTaskParams", diTaskParams)
        self.assertTrue('niDAQmx::setParam - diTaskParams. diTaskParams is not implemented' in str(context.exception))

    def test_calling_doTaskParams_not_yet_implemented(self):
        strings = param_strings
        doTaskParams = strings["doTaskParamsSR"] + strings["doTaskParamsS"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("doTaskParams", doTaskParams)
        self.assertTrue('niDAQmx::setParam - doTaskParams. doTaskParams is not implemented' in str(context.exception))

    def test_calling_ciTaskParams_not_yet_implemented(self):
        strings = param_strings
        ciTaskParams = strings["ciTaskParamsSR"] + strings["ciTaskParamsS"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("ciTaskParams", ciTaskParams)
        self.assertTrue('niDAQmx::setParam - ciTaskParams. ciTaskParams is not implemented' in str(context.exception))

    def test_calling_coTaskParams_not_yet_implemented(self):
        strings = param_strings
        coTaskParams = strings["coTaskParamsSR"] + strings["coTaskParamsS"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("coTaskParams", coTaskParams)
        self.assertTrue('niDAQmx::setParam - coTaskParams. coTaskParams is not implemented' in str(context.exception))

if __name__ == '__main__':
    unittest.main()
