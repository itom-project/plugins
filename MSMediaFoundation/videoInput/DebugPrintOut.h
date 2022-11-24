#pragma once

/// Class for printing info into console
class DebugPrintOut
{
public:
    DebugPrintOut();

    ~DebugPrintOut();

    void printOut(const char *format, ...);

    void setVerbose(bool state);

private:
    bool m_verbose;
        
};

