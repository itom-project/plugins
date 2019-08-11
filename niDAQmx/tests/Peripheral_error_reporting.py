import unittest
from base_data import param_strings
from itom import dataIO
from itom import dataObject

class PeripheralTests(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.plugin=dataIO("niDAQmx")

    @classmethod
    def tearDownClass(self):
        del self.plugin

    def test_invalid_config_mode_caught_in_niAnalogInputChannel_applyParameters(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        self.plugin.setParam("configForTesting","passThroughToPeripheralClasses,ignoreTaskParamInit")
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + "5" + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]
        self.plugin.setParam("aiTaskParams", aiTaskParams)             
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiChParams", aiChParams)
        self.assertTrue('NiAnalogInputChannel::applyParameters: Configmode' in str(context.exception))

    def test_set_channel_params_before_setting_task_params_caught_in_niAnalogInputChannel_applyParameters(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        self.plugin.setParam("configForTesting","ignoreTaskParamInit")
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]                    
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiChParams", aiChParams)
        self.assertTrue('NiAnalogInputChannel::applyParameters: task parameters must be set before setting channel parameters.' in str(context.exception))

    def test_try_to_add_channel_to_task_twice(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"]
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]                    
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        self.plugin.setParam("aiChParams", aiChParams)
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiChParams", aiChParams)
        self.assertTrue('NiAnalogInputChannel::applyParameters: NI routine reported a create channel abnormality -' in str(context.exception))

    def test_invalid_task_mode_caught_in_NiTask_applyParameters(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        self.plugin.setParam("configForTesting","passThroughToPeripheralClasses,ignoreTaskParamInit")
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + "3"
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiChParams", aiChParams)
        self.assertTrue('NiTask::applyParameters: Task mode' in str(context.exception))

    def test_invalid_trigger_channel_specified_in_task_parameters(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        strings = param_strings
        aiTaskParams = strings["aiTaskParamsSR"] + "," + strings["aiTaskParamsS"] + "," + strings["aiTaskParamsMFinite"] + ", /Dev1/PFI2, rising"
        self.plugin.setParam("aiTaskParams", aiTaskParams)
        aiChParams = strings["aiChParamsDev"] + strings["aiChParamsCh"] + "," + strings["aiChParamsMNRSE"] + "," + strings["aiChParamsMinOutputLim"] + "," + strings["aiChParamsMaxOutputLim"]
        with self.assertRaises(RuntimeError) as context:
            self.plugin.setParam("aiChParams", aiChParams)
        self.assertTrue('NiTask::applyParameters: NI function returned configure channel abnormality' in str(context.exception))

    def test_niTaskRun_run_catch_task_uninitialized(self):
        # get pristine state
        self.tearDownClass()
        self.setUpClass()

        self.plugin.setParam("configForTesting","ipassThroughToPeripheralClasses")
        with self.assertRaises(RuntimeError) as context:
            self.plugin.acquire(1)
        self.assertTrue('NiTask::run: Task cannot be started, since it is not initialized' in str(context.exception))

if __name__ == '__main__':
    unittest.main()
