#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <iostream>

#include "DebugPrintOut.h"


//-----------------------------------------------------------
DebugPrintOut::DebugPrintOut(void):
    m_verbose(false)
{
}

//-----------------------------------------------------------
DebugPrintOut::~DebugPrintOut(void)
{
}

//-----------------------------------------------------------
void DebugPrintOut::printOut(const char *format, ...)
{
    if (m_verbose)
    {
        va_list args;

        va_start(args, format);

        char buffer[256];

        vsprintf_s(buffer, 255, format, args);

        std::cout << buffer << std::endl;

        va_end (args);
    }
}

//-----------------------------------------------------------
void DebugPrintOut::setVerbose(bool state)
{
    m_verbose = state;
}