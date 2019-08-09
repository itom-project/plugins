change_as_needed =  {
                "aiTaskParamsSR" : "20000",
                "aiTaskParamsS" : "100",
                "aiTaskParamsTCDev" : "/Dev1",
                "aiTaskParamsTCPFI" : "/PFI",
                "aoTaskParamsSR" : "Dev1",
                "aoTaskParamsS" : "/ao0",
                "diTaskParamsSR" : "Dev1",
                "diTaskParamsS" : "/di0",
                "doTaskParamsSR" : "Dev1",
                "doTaskParamsS" : "/do0",
                "ciTaskParamsSR" : "Dev1",
                "ciTaskParamsS" : "/ci0",
                "coTaskParamsSR" : "Dev1",
                "coTaskParamsS" : "/co0",
                "aiChParamsDev" : "Dev1",
                "aiChParamsCh" : "/ai0",
                "aiChParamsMinOutputLim" : "-10",
                "aiChParamsMaxOutputLim" : "10",
                "aoChParamsDev" : "Dev1",
                "aoChParamsCh" : "/ao0",
                "aoChParamsMinOutputLim" : "-10",
                "aoChParamsMaxOutputLim" : "10",
                "diChParamsDev" : "Dev1",
                "diChParamsCh" : "/di0",
                "doChParamsDev" : "Dev1",
                "doChParamsCh" : "/do0",
                "ciChParamsDev" : "Dev1",
                "ciChParamsCh" : "/ci0",
                "coChParamsDev" : "Dev1",
                "coChParamsCh" : "/co0"
                }

do_not_change =  {
                "aiTaskParamsMFinite" : "0",
                "aiTaskParamsMContinuous" : "1",
                "aiTaskParamsMOnDemand" : "2",
                "aiChParamsMDefault" : "0",
                "aiChParamsMDiff" : "1",
                "aiChParamsMRSE" : "2",
                "aiChParamsMNRSE" : "3",
                "aiChParamsMPseudoDiff" : "4"
                }

param_strings = {**do_not_change, **change_as_needed}


