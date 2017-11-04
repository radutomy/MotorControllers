using System;
using System.IO.Ports;

namespace MotorControllers.FestoMotor
{
   public class FestoMotor : IFestoFunctions
   {
      private readonly SerialPort _comPort;

      private int _endTicks = 0;                           // used to measure time
      private bool _dataReceived = false;                  // turns true when SerialData Received event risen 
      private bool _comEventSet = false;                   // monitors if the SerialData Event Handler has been attached
      private int _watchDog = 0;                           // counter
      private string _message;                             // string that contains the serial response

      // extract bits from byte
      private bool IsBitSet(byte b, int pos) => ((Int32)b & (1 << pos)) != 0;

      public FestoMotor(ushort portNum)
      {
         if (portNum < 1)
            throw new ArgumentOutOfRangeException(nameof(portNum) + " cannot be less than zero");

         _comPort = new SerialPort("COM" + portNum, 9600, Parity.None, 8, StopBits.One);
      }

      public void Home()
      {
         throw new System.NotImplementedException();
      }

      public PrgSts Enable()
      {
         PrgSts prgHomeFesto = new PrgSts();

         //evUpdatePhase(MachinePhase.Homing);
         // data received event handler for Com2
         if (!_comEventSet)
         {
            _comEventSet = true;
            _comPort.DataReceived += new SerialDataReceivedEventHandler(DataReceived);
         }
         try
         {
            // Open the port for communications
            _comPort.Open();
            //if (_debugFesto)
            {
               Console.WriteLine("0) " + _comPort.PortName + "<> PORT OPEN");
            }
            // reset communication variables
            prgHomeFesto.State = Status.Processing;
         }
         catch (Exception e)
         {
            prgHomeFesto.Message = "0) Port Failed to Open";
            prgHomeFesto.State = Status.Failed;
            //if (MainProg.bDebugFesto)
            {
               Console.WriteLine(_comPort.PortName + " - " + _message + " - " + e.ToString());
            }
         }

         return prgHomeFesto;
      }

      public PrgSts Disable()
      {
         throw new System.NotImplementedException();
      }

      public PrgSts SetSpeed(double speed)
      {
         throw new System.NotImplementedException();
      }
   }
}