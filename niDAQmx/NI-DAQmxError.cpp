/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "NI-DAQmxError.h"


ito::RetVal niDAQmxError::checkError(int error, const QString &prefix = "")
{
    ito::RetVal retValue(ito::retOk);

    if (error != DAQmxSuccess)
    {
        // get size of buffer for error string and allocate buffer
        int buffer_size = DAQmxGetErrorString(error, 0, 0);
        char * buffer = new char[buffer_size + 1];

        // Get string text corresponding to error code
        DAQmxGetErrorString(error, buffer, buffer_size);
        buffer[buffer_size] = '\0';

        
	
	    if (!DAQmxFailed(error))
	    {
	        if (prefix != "")
	        {
		        retValue += ito::RetVal::format(ito::retWarning, 0, "%s: %s", prefix.toLatin1().constData(), buffer);
	        }
	        else
	        {
		        retValue += ito::RetVal::format(ito::retWarning, 0, "%s", buffer);
	        }
	    }
	    else
	    {
            int buffer_size_extended = DAQmxGetExtendedErrorInfo(0, 0);
            char *buffer_extended = new char[buffer_size_extended + 1];

            DAQmxGetExtendedErrorInfo(buffer_extended, buffer_size_extended);
            buffer_extended[buffer_size_extended] = '\0';

            char *buffer_final = buffer_size_extended > 0 ? buffer_extended : buffer;

	        if (prefix != "")
	        {
		        retValue += ito::RetVal::format(ito::retError, 0, "%s: %s", prefix.toLatin1().constData(), buffer_final);
	        }
	        else
	        {
		        retValue += ito::RetVal::format(ito::retError, 0, "%s", buffer_final);
	        }

            delete[] buffer_extended;
	    }

	    delete[] buffer;
    }

    return retValue;
}
