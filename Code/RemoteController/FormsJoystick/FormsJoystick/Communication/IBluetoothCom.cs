using System;

namespace FormsJoystick.Communication
{
    public interface IBluetoothCom
    {
        bool FindDevice(string deviceName);
        bool Connect();
        void Close();
        bool SendData(byte command);
    }
}
