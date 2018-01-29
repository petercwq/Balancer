using System;
using System.Threading.Tasks;

namespace FormsJoystick.Communication
{
    public interface IBluetoothCom
    {
        bool FindDevice(string deviceName);
        Task<bool> ConnectAsync();
        void Close();
        bool SendData(byte command);
    }
}
