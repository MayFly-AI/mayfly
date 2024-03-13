""" Utility methods for discovering serial devices """
import sys
import glob
import itertools

def _find_serial_win():
    """ Windows specific version, traversing registry for finding COM ports """
    import winreg
    path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
    key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
    ports = []
    for i in itertools.count():
        try:
            ports.append(winreg.EnumValue(key, i)[1])
        except EnvironmentError:
            break
    return ports

def find_serial_device():
    """ Attemps to find a list of usual suspected serial devices,
        depends on OS. In the case of multiple candidates this will 
        return the first candidate found by the underlying method - glob or winreg.
    """
    found = []
    if sys.platform == 'darwin':
        pattern = '/dev/cu.usbmodem*'
        found = glob.glob(pattern)
    elif sys.platform == 'linux':
        pattern = '/dev/ttyACM*'
        found = glob.glob(pattern)
    elif sys.platform == 'win32':
        found = _find_serial_win()
    else:
        print(f'Unknown platform: {sys.platform}')
        return None

    if len(found) == 0:
        print(f'No devices matching: {pattern}')
        return None

    if len(found) > 1:
        print('Multiple devices found:')
        print(found)
    return found[0]

if __name__ == '__main__':
    dev = find_serial_device()
    print('device = ', dev)
