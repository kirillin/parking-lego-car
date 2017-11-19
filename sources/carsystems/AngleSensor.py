from ev3dev.auto import *
import os
import pyudev


class AngleSensor:
    """
        Only for HT-NXT-ANGLE-SENSOR!
        Auto connection! Don't need specify number of connection port!
        Default mode of measuring is angle.
    """
    def __init__(self):
        """Auto searching connected HT angle sensor"""
        devices = list(pyudev.Context().list_devices(subsystem='lego-sensor').match_property("LEGO_DRIVER_NAME", 'ht-nxt-angle'))

        if not devices:
            raise Exception('AngleSensor not found')

        if devices[0].attributes['driver_name'] == b'ht-nxt-angle':
            self.device = devices[0]
        else:
            raise Exception('Problems with HT-NTX-ANGLE sensor. Try reconnect it...')

        self.setMode('ANGLE')
        self.value0 = open(os.path.join(self.device.sys_path, 'value0'), 'r')

    def setMode(self, value):
        with open(os.path.join(self.device.sys_path, 'mode'), 'w') as f:
            f.write(str(value))

    def getAngle(self):
        self.value0.seek(0)
        return int(self.value0.read())
