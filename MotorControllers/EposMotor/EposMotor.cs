using System;
using System.IO.Ports;

namespace MotorControllers.EposMotor
{
   public class EposMotor : IMotorBasicFunctions
   {
      // serial port object with the appropriate settings
      private SerialPort _com1;
      
      int _endTicks = 0;                           // used to measure time
      ushort _statusWord;                          // stores the last response from a Read request
      bool _dataReceived = false;                  // turns true when SerialData Received event risen        
      string _systemStatus;                        // used for system diagnostics
      bool _com1EventSet = false;

      // instance constructor
      public EposMotor(SerialPort _com1)
      {
         _com1 = new SerialPort("_com1", 9600, Parity.None, 8, StopBits.One);
      }

      // Read/Write states
      private class RWSts : PrgSts
      {
         
      }

      #region FormatReadMessage
      // FormatReadMessage receives an index and subindex and returns a byte array containing a specific instruction that can be written to the EPOS
      private byte[] FormatReadMessage(Int16 index, byte subindex)
      {
         byte[] MessageToSend = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 10 bytes

         MessageToSend[0] = 0x10;                            // write mode
         MessageToSend[1] = 0x01;                            // length-1  
         MessageToSend[2] = (byte)(index & 255);             // indexbyte1 -> extracted first set of 8 bits from index int
         MessageToSend[3] = (byte)((index >> 8) & 255);      // indexbyte2 -> extracted second set of 8 bits from index int
         MessageToSend[4] = subindex;                        // subindex
         MessageToSend[5] = 0x02;                            // node ID

         ushort CRC = CalcFieldCRC(MessageToSend, 4);
         MessageToSend[6] = (byte)CRC;                       // CRC Low byte
         MessageToSend[7] = (byte)(CRC / 256);               // CRC High Byte
         MessageToSend[8] = 0x4F;                            // acknowledgement (ASCII "O")
         MessageToSend[9] = 0x46;                            // failed acknowledgement (ASCII "F")
         return MessageToSend;
      }
      #endregion FormatReadMessage

      #region FormatWriteMessage

      // FormatWriteMessage receives an index and data and returns a byte array containing a specific instruction that can be written to the EPOS
      private byte[] FormatWriteMessage(Int16 index, Int32 data)
      {
         byte[] byMessageToSend = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 14 bytes

         byMessageToSend[0] = 0x11;                            // write mode
         byMessageToSend[1] = 0x03;                            // length-1  
         byMessageToSend[2] = (byte)(index & 255);             // indexbyte1 -> extracted first set of 8 bits from the index int
         byMessageToSend[3] = (byte)((index >> 8) & 255);      // indexbyte2 -> extracted second set of 8 bits from the index int
         byMessageToSend[4] = 0x00;                            // subindex
         byMessageToSend[5] = 0x02;                            // node ID
         byMessageToSend[6] = (byte)(data & 255);              // databyte1 -> extracted first set of 8 bits from int
         byMessageToSend[7] = (byte)((data >> 8) & 255);       // databyte2 -> extracted second set of 8 bits from int
         byMessageToSend[8] = (byte)((data >> 16) & 255);      // databyte3 -> extracted third set of 8 bits from int
         byMessageToSend[9] = (byte)((data >> 24) & 255);      // databyte4 -> extracted fourth set of 8 bits from int

         ushort CRC = CalcFieldCRC(byMessageToSend, 6);
         byMessageToSend[10] = (byte)CRC;                      // CRC Low byte
         byMessageToSend[11] = (byte)(CRC / 256);              // CRC High Byte
         byMessageToSend[12] = 0x4F;                           // acknowledgement (ASCII "O")
         byMessageToSend[13] = 0x46;                           // failed acknowledgement (ASCII "F")            
         return byMessageToSend;
      }

      #endregion FormatWriteMessage

      #region CRC

      //Maxon CRC check converted from C++ example
      ushort CalcFieldCRC(byte[] ByteArray, ushort numberOfWords)
      {
         ushort[] nWordArray = { 0, 0, 0, 0, 0, 0 };
         int iDataX;
         int iDataY;
         //Decode bytes into word array for CRC check
         iDataX = ByteArray[1];
         iDataY = ByteArray[0] * 256;
         nWordArray[0] = (ushort)(iDataX + iDataY);
         iDataX = ByteArray[2];
         iDataY = ByteArray[3] * 256;
         nWordArray[1] = (ushort)(iDataX + iDataY);
         iDataX = ByteArray[4];
         iDataY = ByteArray[5] * 256;
         nWordArray[2] = (ushort)(iDataX + iDataY);
         iDataX = ByteArray[6];
         iDataY = ByteArray[7] * 256;
         nWordArray[3] = (ushort)(iDataX + iDataY);
         if (numberOfWords > 4)
         {
            iDataX = ByteArray[8];
            iDataY = ByteArray[9] * 256;
            nWordArray[4] = (ushort)(iDataX + iDataY);
            iDataX = ByteArray[10];
            iDataY = ByteArray[11] * 256;
            nWordArray[5] = (ushort)(iDataX + iDataY);
         }
         ushort shifter;
         ushort data = 0;
         ushort carry = 0;
         ushort CRC = 0;
         ushort mask = 0x8000;
         //Calculate WordArray Word by Word            
         for (ushort WordNum = 0; WordNum <= (numberOfWords - 1); WordNum++)
         {
            shifter = 0x8000;
            data = nWordArray[WordNum];

            do
            {
               carry = (ushort)(CRC & mask);
               //CRC = CRC * 2
               CRC <<= 1;
               if ((data & shifter) > 0)
               {
                  //CRC = CRC + 1, if BitX is set in c
                  CRC++;
               }
               if (carry > 0)
               {
                  //CRC = CRC XOR G(x), if carry is true
                  CRC ^= 0x1021;
               }
               //Set BitX to next lower Bit, shifter = shifter/2
               shifter >>= 1;
            } while (shifter > 0);
         }
         return CRC;
      }
      #endregion CRCCalculation

      #region ReceiveData

      private RWSts ReceiveData(byte[] aRS232set, RWSts Receive)
      {
         // dynamicly re-sizeable array based on the amount of bytes in the serial receive buffer
         byte[] byRS232buffer = new byte[_com1.BytesToRead];

         switch (Receive.Step)
         {
            // initialise variables used for communication
            case 0:
               Receive.State = Status.Processing;
               Receive.Step = 1;
               break;

            case 1:
               //Send 1st byte to EPOS2, this is the OPCode (0x10 = Read, 0X11 = Write)
               _com1.Write(aRS232set, 0, 1);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               // Go to next step 
               Receive.Step = 2;
               break;

            case 2:
               // Wait for event handler data recieved
               if (_dataReceived)
               {
                  Receive.Step = 3;
               }
               if (Environment.TickCount > _endTicks)
               {
                  // No response within 1 second
                  Receive.Step = 40;
               }
               break;

            case 3:
               // Read the response from the EPOS                          
               _com1.Read(byRS232buffer, 0, _com1.BytesToRead);
               // Reset received flag
               _dataReceived = false;
               Receive.Step = 4;
               break;

            case 4:
               // Write request string to read the software version
               _com1.Write(aRS232set, 1, 7);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               Receive.Step = 5;
               break;

            case 5:
               //Wait for event handler to say data recieved
               if (_dataReceived)
               {
                  Receive.Step = 6;
               }
               if (Environment.TickCount > _endTicks)
               {
                  // No response within 1 second
                  Receive.Step = 40;
               }
               break;

            case 6:
               // Receive data from buffer
               _com1.Read(byRS232buffer, 0, _com1.BytesToRead);
               _dataReceived = false;
               // Check if the response back is an 'O' character to say data request was received OKEn
               if (byRS232buffer[0] == 79)
               {
                  Receive.Step = 7;
               }
               else
               {
                  // error in reading response from the Epos
                  Receive.Step = 30;
               }

               break;

            case 7:
               // write acknowledgement
               _com1.Write(aRS232set, 8, 1);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               Receive.Step = 8;
               break;

            case 8:
               //Wait for event handler data recieved
               if (_dataReceived)
               {
                  Receive.Step = 9;
               }
               if (Environment.TickCount > _endTicks)
               {
                  // No response within 1 second
                  Receive.Step = 40;
               }
               break;

            case 9:
               // Receive data from the buffer
               _com1.Read(byRS232buffer, 0, _com1.BytesToRead);
               // reset data received flag            
               try
               {
                  // interpret the reponse from the controller -> extract the 5th and 6th byte and convert to a word
                  _statusWord = (ushort)((((ushort)(byRS232buffer[6] * 256)) + ((ushort)byRS232buffer[5]) & 0x417F));
               }
               catch (Exception e)
               {
                  // exception error handling
                  _systemStatus = e.ToString();
                  // Control word could not be interpreted
                  Receive.Step = 30;
                  break;
               }
               _dataReceived = false;
               Receive.Step = 10;
               break;

            case 10:
               // write acknowledgement (character "O")
               _com1.Write(aRS232set, 8, 1);

               // Report successful finish
               Receive.Step = 20;
               break;

            // Operation Finished sucessfully
            case 20:
               Receive.Step = 0;
               Receive.State = Status.FinishedOk;
               Receive.Message = "Operation Finished Sucessfully";
               break;

            // Communications error received
            case 30:
               Receive.Step = 0;
               Receive.State = Status.Failed;
               Receive.Message = "Communication Error Received";
               break;

            // Operation timed out
            case 40:
               Receive.Step = 0;
               Receive.State = Status.TimedOut;
               Receive.Message = "Operation Timed Out";
               break;
         }
         return Receive;
      }
      #endregion RequestData

      #region SendData


      private RWSts SendData(byte[] aRS232set, RWSts Send)
      {
         // buffer that stores the data received back from the EPOS           

         if (_com1.IsOpen)
         {
            byte[] RS232buffer = new byte[_com1.BytesToRead];
            switch (Send.Step)
            {
               // initialise variables used for communication
               case 0:
                  Send.Step = 1;
                  Send.State = Status.Processing;
                  break;

               case 1:
                  // send the first byte in the array, representing the op code
                  _com1.Write(aRS232set, 0, 1);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  // Go to next step
                  Send.Step = 2;
                  break;

               case 2:
                  // Wait for event handler data recieved
                  if (_dataReceived)
                  {
                     Send.Step = 3;
                  }
                  if (Environment.TickCount > _endTicks)
                  {
                     // No response within 1 second
                     Send.Step = 40;
                  }
                  break;

               case 3:
                  // Read the response from the EPOS                          
                  _com1.Read(RS232buffer, 0, _com1.BytesToRead);
                  _dataReceived = false;
                  Send.Step = 4;
                  break;

               case 4:
                  // Write request to read  (bytes 1->11 are part of the request)
                  _com1.Write(aRS232set, 1, 11);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  Send.Step = 5;
                  break;

               case 5:
                  // Wait for event handler data recieved
                  if (_dataReceived)
                  {
                     Send.Step = 6;
                  }
                  if (Environment.TickCount > _endTicks)
                  {
                     // No response within 1 second
                     Send.Step = 40;
                  }
                  break;

               case 6:
                  // Read the response from the EPOS                          
                  _com1.Read(RS232buffer, 0, _com1.BytesToRead);
                  _dataReceived = false;
                  // Check if the response back is an 'O' character to say OPCode was received OKEn
                  if (RS232buffer[0] == 79)
                  {
                     Send.Step = 7;
                  }
                  else
                  {
                     // Error was received
                     Send.Step = 30;
                  }
                  break;

               case 7:
                  // send the 12th byte in the array, representing an acknowledgement (character 'O')
                  _com1.Write(aRS232set, 12, 1);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  // Go to next step
                  Send.Step = 8;
                  break;

               case 8:
                  // Wait for event handler data recieved
                  if (_dataReceived)
                  {
                     Send.Step = 9;
                  }
                  if (Environment.TickCount > _endTicks)
                  {
                     // No response within 1 second
                     Send.Step = 40;
                  }
                  break;

               case 9:
                  // Read the response from the EPOS                          
                  _com1.Read(RS232buffer, 0, _com1.BytesToRead);
                  _dataReceived = false;
                  Send.Step = 10;
                  break;

               case 10:
                  // send the 12th byte in the array, representing an acknowledgement (character 'O')
                  _com1.Write(aRS232set, 12, 1);

                  // Operation finished sucessfully
                  Send.Step = 20;
                  break;

               // Operation Finished sucessfully
               case 20:
                  Send.Step = 0;
                  Send.State = Status.FinishedOk;
                  Send.Message = "Operation Finished Sucessfully";
                  break;

               // Communications error received
               case 30:
                  Send.Step = 0;
                  Send.State = Status.Failed;
                  Send.Message = "Communication Error Received";
                  break;

               // Operation timed out
               case 40:
                  Send.Step = 0;
                  Send.State = Status.TimedOut;
                  Send.Message = "Operation Timed Out";
                  break;
            }
         }
         return Send;
      }


      #endregion SendData


      public PrgSts Enable()
      {
         throw new System.NotImplementedException();
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
