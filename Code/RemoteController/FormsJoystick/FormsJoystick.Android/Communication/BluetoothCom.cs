using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using Android.Bluetooth;
using Android.Content;
using FormsJoystick.Communication;
using FormsJoystick.Droid.Communication;
using Java.Lang;
using Java.Util;
using Xamarin.Forms;

[assembly: Dependency(typeof(BluetoothCom))]
namespace FormsJoystick.Droid.Communication
{
    public class BluetoothCom : IBluetoothCom
    {
        // android built in classes for bluetooth operations
        BluetoothAdapter BTAdapter;
        BluetoothSocket BTSocket;
        BluetoothDevice BTDevice;

        // needed for communication to bluetooth device / network
        BinaryWriter BTOutputStream;
        BinaryReader BTInputStream;

        private bool found = false;
        private bool opened = false;

        public BluetoothCom()
        {

        }

        public bool Connected
        {
            get { return opened; }
        }

        // this will find a bluetooth printer device
        public bool FindDevice(string deviceName)
        {

            try
            {
                BTAdapter = BluetoothAdapter.DefaultAdapter;

                if (BTAdapter == null)
                {
                    found = false;
                }

                if (!BTAdapter.IsEnabled)
                {
                    Intent enableBluetooth = new Intent(BluetoothAdapter.ActionRequestEnable);
                    Android.App.Application.Context.StartActivity(enableBluetooth);
                    //BTAdapter.Enable();
                }

                ICollection<BluetoothDevice> pairedDevices = BTAdapter.BondedDevices;

                if (pairedDevices.Count > 0)
                {
                    foreach (BluetoothDevice device in pairedDevices)
                    {
                        if (device.Name == deviceName)//"RPP300"
                        {
                            BTDevice = device;
                            break;
                        }
                    }
                }

                found = true;
                opened = false;
            }
            catch (System.Exception Ex)
            {
                System.Diagnostics.Debug.WriteLine(Ex);
            }
            return found;
        }

        // tries to open a connection to the bluetooth printer device
        public async Task<bool> ConnectAsync()
        {
            try
            {
                if (opened)
                {
                    return true;
                }
                // Standard SerialPortService ID
                UUID uuid = UUID.FromString("00001101-0000-1000-8000-00805f9b34fb");

                try
                {
                    BTSocket = BTDevice.CreateRfcommSocketToServiceRecord(uuid);
                    // CreateInsecureRfcommSocketToServiceRecord
                }
                catch (System.Exception e)
                {
                    System.Diagnostics.Debug.WriteLine(e);
                }

                try
                {
                    await BTSocket.ConnectAsync();
                    opened = true;
                }
                catch (IOException e)
                {
                    System.Diagnostics.Debug.WriteLine(e);
                    try
                    {
                        BTSocket = (BluetoothSocket)BTDevice.Class.GetMethod("createRfcommSocket", new Class[] { uuid.Class }).Invoke(BTDevice, 1);
                        await BTSocket.ConnectAsync();
                        opened = true;
                    }
                    catch (System.Exception e2)
                    {
                        System.Diagnostics.Debug.WriteLine(e2);
                    }
                }
            }
            catch (System.Exception Ex)
            {
                BTSocket.Dispose();
                System.Diagnostics.Debug.WriteLine(Ex);
            }
            if (opened)
            {
                BTOutputStream = new BinaryWriter(BTSocket.OutputStream);
                BTInputStream = new BinaryReader(BTSocket.InputStream);
            }
            return opened;
        }

        // close the connection to bluetooth printer.
        public void Close()
        {
            try
            {
                BTOutputStream.Close();
                BTInputStream.Close();
                BTSocket.Close();
                opened = false;
            }
            catch (System.Exception Ex)
            {
                System.Diagnostics.Debug.WriteLine(Ex);
                opened = false;
            }
        }

        // this will send text data to be printed by the bluetooth printer
        public bool SendData(byte command)
        {
            try
            {
                BTOutputStream.Write(command);
                BTOutputStream.Flush();
                return true;
            }
            catch (System.Exception Ex)
            {
                System.Diagnostics.Debug.WriteLine(Ex);
            }
            return false;
        }

        public bool SendData(byte[] command, int offset = 0, int count = 0)
        {
            try
            {
                BTOutputStream.Write(command, offset > 0 ? offset : 0, count > 0 ? count : command.Length);
                BTOutputStream.Flush();
                return true;
            }
            catch (System.Exception Ex)
            {
                System.Diagnostics.Debug.WriteLine(Ex);
            }
            return false;
        }

        public int ReadData(byte[] buffer, int offset, int count)
        {
            try
            {
                return BTInputStream.Read(buffer, offset, count);
            }
            catch (System.Exception Ex)
            {
                System.Diagnostics.Debug.WriteLine(Ex);
            }
            return 0;
        }
    }
}