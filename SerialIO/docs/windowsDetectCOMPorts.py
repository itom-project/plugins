'''This script can be used as example for Windows
to detect registered COM ports for this computer'''
import winreg as wreg

def DetectCOMPorts():
    try:
        regconn = wreg.ConnectRegistry( None, wreg.HKEY_LOCAL_MACHINE )
        key = wreg.OpenKey( regconn, "HARDWARE\\DEVICEMAP\\SERIALCOMM", wreg.KEY_READ )
        values_count = wreg.QueryInfoKey( key )[1]
        values_list = []
        for i in range( values_count ):
            values_list.append( wreg.EnumValue( key, i ) )
    except ( WindowsError, EnvironmentError ):
        print( "Unable to Connect to the Window Registry and read keys" )
    finally:
        key.Close()
    return values_list

def NumberOfCOMPorts( values_list ):
    for subkey in iter( values_list ):
        print( "Name : " + subkey[0] )
        print( "Data : " + subkey[1] )

NumberOfCOMPorts( DetectCOMPorts() )
