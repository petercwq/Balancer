#region CmdMessenger - MIT - (c) 2013 Thijs Elenbaas.
/*
  CmdMessenger - library that provides command based messaging

  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  Copyright 2013 - Thijs Elenbaas
*/
#endregion

using System;
using System.Collections.Generic;
using System.IO;
using Android.Bluetooth;
using Java.Lang;
using Java.Util;

// Todo: 
// remove isconnected for speedup. 
// Test for disconnected bluetooth

namespace CommandMessenger.Transport.Bluetooth
{
    /// <summary>
    /// Manager for Bluetooth connection
    /// </summary>
    public class BluetoothTransport : ITransport
    {
        private const int BufferSize = 4096;

        private readonly AsyncWorker _worker;
        private readonly object _readLock = new object();
        private readonly object _writeLock = new object();
        private readonly byte[] _readBuffer = new byte[BufferSize];
        private int _bufferFilled;

        // android built in classes for bluetooth operations
        BluetoothSocket BTSocket;
        BluetoothDevice BTDevice;

        // needed for communication to bluetooth device / network
        BinaryWriter BTOutputStream;
        BinaryReader BTInputStream;

        // Event queue for all listeners interested in NewLinesReceived events.
        public event EventHandler DataReceived;

        /// <summary>
        /// Gets or sets Bluetooth device info
        /// </summary>
        public string DeviceName { get; set; }

        /// <summary>
        /// Bluetooth transport constructor
        /// </summary>
        public BluetoothTransport()
        {
            _worker = new AsyncWorker(Poll, "BluetoothTransport");
        }

        /// <summary>
        /// Bluetooth transport destructor
        /// </summary>
        ~BluetoothTransport()
        {
            Disconnect();
        }

        private bool Poll()
        {
            var bytes = UpdateBuffer();
            if (bytes > 0 && DataReceived != null)
                DataReceived(this, EventArgs.Empty);

            return true;
        }

        // this will find a bluetooth printer device
        private bool FindDevice(string deviceName)
        {

            try
            {
                BluetoothAdapter BTAdapter = BluetoothAdapter.DefaultAdapter;

                if (BTAdapter == null)
                {
                    return false;
                }

                if (!BTAdapter.IsEnabled)
                {
                    // Intent enableBluetooth = new Intent(BluetoothAdapter.ActionRequestEnable);
                    // Android.App.Application.Context.StartActivity(enableBluetooth);
                    BTAdapter.Enable();
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
                return true;
            }
            catch (System.Exception Ex)
            {
                System.Diagnostics.Debug.WriteLine(Ex);
            }
            return false;
        }

        /// <summary> Connects to a serial port defined through the current settings. </summary>
        /// <returns> true if it succeeds, false if it fails. </returns>
        public bool Connect()
        {
            // Reconnecting to the same device seems to fail a lot of the time, so see
            // if we can remain connected
            if (BTDevice != null && BTDevice.Name != DeviceName)
            {
                Close();
            }

            if (string.IsNullOrWhiteSpace(DeviceName) || !FindDevice(DeviceName))
                return false;

            try
            {
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
                    BTSocket.Connect();
                }
                catch (IOException e)
                {
                    System.Diagnostics.Debug.WriteLine(e);
                    try
                    {
                        BTSocket = (BluetoothSocket)BTDevice.Class.GetMethod("createRfcommSocket", new Class[] { uuid.Class }).Invoke(BTDevice, 1);
                        BTSocket.Connect();
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

            if( Open())
            {
                // Check worker is not running as a precaution. This needs to be rechecked.
                if (!_worker.IsRunning) _worker.Start();
            }

            return IsOpen();
        }

        /// <summary> Opens the serial port. </summary>
        /// <returns> true if it succeeds, false if it fails. </returns>
        public bool Open()
        {
            if (BTSocket == null || !BTSocket.IsConnected)
                return false;
            lock (_writeLock)
            {
                lock (_readLock)
                {
                    BTOutputStream = new BinaryWriter(BTSocket.OutputStream);
                    BTInputStream = new BinaryReader(BTSocket.InputStream);
                }
            }
            return true;
        }

        /// <summary>
        /// Returns connection status
        /// </summary>
        /// <returns>true when connected</returns>
        public bool IsConnected()
        {
            // If not, test if we are connected
            return (BTSocket != null) && BTSocket.IsConnected;
        }

        /// <summary>
        /// Returns opened stream status
        /// </summary>
        /// <returns>true when open</returns>
        public bool IsOpen()
        {
            // note: this does not always work. Perhaps do a scan
            return IsConnected() && (BTInputStream != null && BTOutputStream!=null);
        }


        /// <summary> Closes the Bluetooth stream port. </summary>
        /// <returns> true if it succeeds, false if it fails. </returns>
        public bool Close()
        {
            lock (_writeLock)
            {
                lock (_readLock)
                {
                    BTOutputStream?.Close();
                    BTInputStream?.Close();
                    BTSocket?.Close();

                    BTOutputStream = null;
                    BTInputStream = null;
                    BTSocket = null;
                    return true;
                }
            }
        }

        /// <summary> Disconnect the bluetooth stream. </summary>
        /// <returns> true if it succeeds, false if it fails. </returns>
        public bool Disconnect()
        {
            // Check worker is running as a precaution. 
            if (_worker.IsRunning) _worker.Stop();
            return Close();
        }

        /// <summary> Writes a byte array to the bluetooth stream. </summary>
        /// <param name="buffer"> The buffer to write. </param>
        public void Write(byte[] buffer)
        {
            try
            {
                if (IsOpen())
                {
                    lock (_writeLock)
                    {
                        BTOutputStream.Write(buffer, 0, buffer.Length);
                    }
                }
            }
            catch (System.Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(ex);
                //Do nothing
            }
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        private int UpdateBuffer()
        {
            if (IsOpen() && BTInputStream.BaseStream.IsDataAvailable())
            {
                try
                {
                    var nbrDataRead = BTInputStream.Read(_readBuffer, _bufferFilled, (BufferSize - _bufferFilled));
                    lock (_readLock)
                    {
                        _bufferFilled += nbrDataRead;
                        //Console.WriteLine("buf: {0}", _bufferFilled.ToString().Length);
                    }
                    return _bufferFilled;
                }
                catch (IOException)
                {
                    //Console.WriteLine("buf: TO");
                    // Timeout (expected)
                }
            }
            else
            {
                // In case of no connection 
                // Sleep a bit otherwise CPU load will go through roof
                System.Threading.Thread.Sleep(25);
            }

            return _bufferFilled;
        }

        /// <summary> Reads the serial buffer into the string buffer. </summary>
        public byte[] Read()
        {
            //if (IsOpen())
            {
                byte[] buffer;
                lock (_readLock)
                {
                    buffer = new byte[_bufferFilled];
                    Array.Copy(_readBuffer, buffer, _bufferFilled);
                    _bufferFilled = 0;
                }
                return buffer;
            }
            //return new byte[0];
        }


        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                Disconnect();
            }
        }
    }
}