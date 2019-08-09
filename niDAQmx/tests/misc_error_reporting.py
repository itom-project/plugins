import unittest
from base_data import param_strings
from itom import dataIO
from itom import dataObject
import time

class miscTests(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.plugin=dataIO("niDAQmx")

    @classmethod
    def tearDownClass(self):
        del self.plugin
    
    def test_startDevice_warning(self):
        with self.assertRaises(RuntimeError) as context:
            self.plugin.startDevice()
        self.assertTrue('niDAQmx::startDevice. Warning: startDevice() is not used in niDAQmx. It is a NOOP' in str(context.exception))
    
    def test_stopDevice_warning(self):
        with self.assertRaises(RuntimeError) as context:
            self.plugin.stopDevice()
        self.assertTrue('niDAQmx::stopDevice. Warning: stopDevice() is not used in niDAQmx It is a NOOP' in str(context.exception))
    
    def test_call_to_acquire_twice_without_intervening_call_to_getVal_or_copyVal(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + ","  + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.plugin.setParam("aiChParams", aiChParams)
        self.plugin.acquire(1)
        with self.assertRaises(RuntimeError) as context:
            self.plugin.acquire(1)
        self.assertTrue('niDAQmx::acquire - acquire was called twice without an intervening call to getVal or copyVal' in str(context.exception))

    def test_call_copyVal_with_null_pointer(self):   
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + ","  + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.plugin.setParam("aiChParams", aiChParams)             
        self.plugin.acquire(1)
        with self.assertRaises(RuntimeError) as context:
            self.plugin.copyVal(0)
        self.assertTrue('niDAQmx::copyVal - Empty object handle provided by caller' in str(context.exception))
    
    def test_call_copyVal_before_calling_acquire(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + ","  + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.plugin.setParam("aiChParams", aiChParams)             
        d=dataObject()
        with self.assertRaises(RuntimeError) as context:
            self.plugin.copyVal(d)
        self.assertTrue('niDAQmx::retrieveData - cannot retrieve data because none was acquired.' in str(context.exception))

    def test_call_setVal_with_empty_dataObject(self):
        d=dataObject()
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setVal(d)
        self.assertTrue('niDAQmx::setVal - Error occured: given Dataobject has wrong dimensions.' in str(context.exception))

    def test_exec_call_with_invalid_function_name(self):
        with self.assertRaises(RuntimeError) as context:
            self.plugin.exec("none")
        self.assertTrue("plugin does not provide an execution of function 'none'" in str(context.exception))

    def test_check_data_with_invalid_dataObject(self):   
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + ","  + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.plugin.setParam("aiChParams", aiChParams)             
        self.plugin.acquire(1)
        d=dataObject([5,3], "int8");
        with self.assertRaises(RuntimeError) as context:
            self.plugin.copyVal(d)
        self.assertTrue('niDAQmx::checkData - Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.' in str(context.exception))

    def test_check_data_with_dataObject_of_too_many_planes(self):   
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + ","  + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.plugin.setParam("aiChParams", aiChParams)             
        self.plugin.acquire(1)
        d=dataObject([5,3,2,2], "int8");
        with self.assertRaises(RuntimeError) as context:
            self.plugin.copyVal(d)
        self.assertTrue('niDAQmx::checkData - Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.' in str(context.exception))

if __name__ == '__main__':
    unittest.main()
