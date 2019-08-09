import unittest
from base_data import param_strings
from itom import dataIO
from itom import dataObject

class ReadChannelFinite(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.plugin=dataIO("niDAQmx")

    @classmethod
    def tearDownClass(self):
        del self.plugin
    
    def test_get_set_task_parameters(self):
        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)             
        aiTaskParamsOutput = self.plugin.getParam("aiTaskParams")
        self.assertTrue(aiTaskParams == aiTaskParamsOutput)

    def test_get_set_channel_parameters(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiChParams", aiChParams)             
        aiChParamsOutput = self.plugin.getParam("aiChParams")
        self.assertTrue(aiChParams == aiChParamsOutput)

    def test_acquire(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiChParams", aiChParams)             
        self.plugin.acquire()

# The following unittest only tests that the call sequence to acquire data doesn't return an error. It doesn't test that the data returned
# conforms to some predefined value. That would be hard to do with a standard unittest, since there is no way (at least on Linux) to
# attach a channel to a source of predefined data. Testing that the data returned is valid must be done outside the unittest framework.

    def test_finite_read_channel(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiChParams", aiChParams)             
        self.plugin.acquire(1)
        self.d=dataObject()
        self.plugin.getVal(self.d)
        
if __name__ == '__main__':
    unittest.main()
