using System;
using System.ComponentModel;
using CommandMessenger.Transport.Bluetooth;
using CommandMessenger;
using CommandMessenger.Queue;

namespace FormsJoystick.ViewModels
{
    public enum Commands:int
    {
        Acknowledge=0,
        Error,
        SetParams, // Command to request led to be set in specific state
        GetParams,
        Move,
        Calibrate,
        GetBattery
    }

    public abstract class BaseViewModel : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        protected void NotifyPropertyChanged(string name)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }

        private static BluetoothTransport _bluetoothTransport;
        private static CmdMessenger _cmdMessenger;


        static BaseViewModel()
        {
            // Create Serial Port object
            // Note that for some boards (e.g. Sparkfun Pro Micro) DtrEnable may need to be true.
            _bluetoothTransport = new BluetoothTransport
            {
                DeviceName = "EMS01_1152"
            };

            // Initialize the command messenger with the Serial Port transport layer
            // Set if it is communicating with a 16- or 32-bit Arduino board
            _cmdMessenger = new CmdMessenger(_bluetoothTransport, BoardType.Bit16);

            // Attach to NewLinesReceived for logging purposes
            _cmdMessenger.NewLineReceived += NewLineReceived;

            // Attach to NewLineSent for logging purposes
            _cmdMessenger.NewLineSent += NewLineSent;


            _cmdMessenger.Attach(OnUnknownCommand);
            _cmdMessenger.Attach((int)Commands.Acknowledge, OnAcknowledge);
            _cmdMessenger.Attach((int)Commands.Error, OnError);
        }

        // Called when a received command has no attached function.
        // In a WinForm application, console output gets routed to the output panel of your IDE
        static void OnUnknownCommand(ReceivedCommand arguments)
        {
            Console.WriteLine(@"Command without attached callback received");
        }

        // Callback function that prints that the Arduino has acknowledged
        static void OnAcknowledge(ReceivedCommand arguments)
        {
            Console.WriteLine($"ACK: {arguments.ReadStringArg()}");
        }

        // Callback function that prints that the Arduino has experienced an error
        static void OnError(ReceivedCommand arguments)
        {
            Console.WriteLine($"Error: {arguments.ReadStringArg()}");
        }

        // Log received line to console
        private static void NewLineReceived(object sender, CommandEventArgs e)
        {
            Console.WriteLine(@"Received > " + e.Command.CommandString());
        }

        // Log sent line to console
        private static void NewLineSent(object sender, CommandEventArgs e)
        {
            Console.WriteLine(@"Sent > " + e.Command.CommandString());
        }

        protected static bool IsConnected{
            get{
                return _bluetoothTransport.IsConnected();
            }
        }

        protected bool Connect()
        {
            // Start listening
            return _cmdMessenger.Connect();
        }

        protected void Disconnect()
        {
            // Stop listening
            _cmdMessenger.Disconnect();
        }

        protected void Dispose()
        {
            _cmdMessenger.Dispose();
            _bluetoothTransport.Dispose();
        }

        /// Attach command call backs. 
        protected void AttachCommandAction(Commands cmd, Action<ReceivedCommand> receivedAction)
        {
            _cmdMessenger.Attach((int)cmd, args => receivedAction(args));
        }

        // Sent command to change led blinking frequency
        protected void QueueCommand(SendCommand cmd)
        {
            // Put the command on the queue and wrap it in a collapse command strategy
            // This strategy will avoid duplicates of this certain command on the queue: if a SetLedFrequency command is
            // already on the queue when a new one is added, it will be replaced at its current queue-position. 
            // Otherwise the command will be added to the back of the queue. 
            // 
            // This will make sure that when the slider raises a lot of events that each send a new blink frequency, the 
            // embedded controller will not start lagging.
            _cmdMessenger.QueueCommand(new CollapseCommandStrategy(cmd));
        }


        // Sent command to change led on/of state
        protected void SendCommand(SendCommand cmd)
        {
            // Send command
            _cmdMessenger.SendCommand(cmd);
        }
    }
}
