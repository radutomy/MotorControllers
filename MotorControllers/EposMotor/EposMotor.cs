using System;
using System.IO.Ports;
using System.Runtime.Remoting.Messaging;

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

      public EposMotor(ushort portNum)
      {
         if (portNum < 1)
            throw new ArgumentOutOfRangeException(nameof(portNum) + " cannot be zero");
         
         _com1 = new SerialPort("COM" + portNum, 9600, Parity.None, 8, StopBits.One);
      }

      // Read/Write states
      private class RWSts : PrgSts
      {
         
      }

      #region FormatReadMessage
      // FormatReadMessage receives an index and subindex and returns a byte array containing a specific instruction that can be written to the EPOS
      private byte[] FormatReadMessage(Int16 index, byte subindex)
      {
         byte[] msgSent = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 10 bytes

         msgSent[0] = 0x10;                            // write mode
         msgSent[1] = 0x01;                            // length-1  
         msgSent[2] = (byte)(index & 255);             // indexbyte1 -> extracted first set of 8 bits from index int
         msgSent[3] = (byte)((index >> 8) & 255);      // indexbyte2 -> extracted second set of 8 bits from index int
         msgSent[4] = subindex;                        // subindex
         msgSent[5] = 0x02;                            // node ID

         ushort CRC = CalcFieldCRC(msgSent, 4);
         msgSent[6] = (byte)CRC;                       // CRC Low byte
         msgSent[7] = (byte)(CRC / 256);               // CRC High Byte
         msgSent[8] = 0x4F;                            // acknowledgement (ASCII "O")
         msgSent[9] = 0x46;                            // failed acknowledgement (ASCII "F")
         return msgSent;
      }
      #endregion FormatReadMessage

      #region FormatWriteMessage

      // FormatWriteMessage receives an index and data and returns a byte array containing a specific instruction that can be written to the EPOS
      private byte[] FormatWriteMessage(Int16 index, Int32 data)
      {
         byte[] msgSent = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 14 bytes

         msgSent[0] = 0x11;                            // write mode
         msgSent[1] = 0x03;                            // length-1  
         msgSent[2] = (byte)(index & 255);             // indexbyte1 -> extracted first set of 8 bits from the index int
         msgSent[3] = (byte)((index >> 8) & 255);      // indexbyte2 -> extracted second set of 8 bits from the index int
         msgSent[4] = 0x00;                            // subindex
         msgSent[5] = 0x02;                            // node ID
         msgSent[6] = (byte)(data & 255);              // databyte1 -> extracted first set of 8 bits from int
         msgSent[7] = (byte)((data >> 8) & 255);       // databyte2 -> extracted second set of 8 bits from int
         msgSent[8] = (byte)((data >> 16) & 255);      // databyte3 -> extracted third set of 8 bits from int
         msgSent[9] = (byte)((data >> 24) & 255);      // databyte4 -> extracted fourth set of 8 bits from int

         ushort CRC = CalcFieldCRC(msgSent, 6);
         msgSent[10] = (byte)CRC;                      // CRC Low byte
         msgSent[11] = (byte)(CRC / 256);              // CRC High Byte
         msgSent[12] = 0x4F;                           // acknowledgement (ASCII "O")
         msgSent[13] = 0x46;                           // failed acknowledgement (ASCII "F")            
         
         return msgSent;
      }

      #endregion FormatWriteMessage

      #region CRC

      //Maxon CRC check converted from C++ example
      ushort CalcFieldCRC(byte[] byteArray, ushort numOfWords)
      {
         ushort[] nWordArray = { 0, 0, 0, 0, 0, 0 };
         int iDataX;
         int iDataY;
         //Decode bytes into word array for CRC check
         iDataX = byteArray[1];
         iDataY = byteArray[0] * 256;
         nWordArray[0] = (ushort)(iDataX + iDataY);
         iDataX = byteArray[2];
         iDataY = byteArray[3] * 256;
         nWordArray[1] = (ushort)(iDataX + iDataY);
         iDataX = byteArray[4];
         iDataY = byteArray[5] * 256;
         nWordArray[2] = (ushort)(iDataX + iDataY);
         iDataX = byteArray[6];
         iDataY = byteArray[7] * 256;
         nWordArray[3] = (ushort)(iDataX + iDataY);
         if (numOfWords > 4)
         {
            iDataX = byteArray[8];
            iDataY = byteArray[9] * 256;
            nWordArray[4] = (ushort)(iDataX + iDataY);
            iDataX = byteArray[10];
            iDataY = byteArray[11] * 256;
            nWordArray[5] = (ushort)(iDataX + iDataY);
         }
         ushort shifter;
         ushort data = 0;
         ushort carry = 0;
         ushort CRC = 0;
         ushort mask = 0x8000;
         //Calculate WordArray Word by Word            
         for (ushort wordNum = 0; wordNum <= (numOfWords - 1); wordNum++)
         {
            shifter = 0x8000;
            data = nWordArray[wordNum];

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

      private RWSts ReceiveData(byte[] rs232Set, RWSts receive)
      {
         // dynamicly re-sizeable array based on the amount of bytes in the serial receive buffer
         byte[] rs232Buff = new byte[_com1.BytesToRead];

         switch (receive.Step)
         {
            // initialise variables used for communication
            case 0:
               receive.State = Status.Processing;
               receive.Step = 1;
               break;

            case 1:
               //send 1st byte to EPOS2, this is the OPCode (0x10 = Read, 0X11 = Write)
               _com1.Write(rs232Set, 0, 1);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               // Go to next step 
               receive.Step = 2;
               break;

            case 2:
               // Wait for event handler data recieved
               if (_dataReceived)
               {
                  receive.Step = 3;
               }
               if (Environment.TickCount > _endTicks)
               {
                  // No response within 1 second
                  receive.Step = 40;
               }
               break;

            case 3:
               // Read the response from the EPOS                          
               _com1.Read(rs232Buff, 0, _com1.BytesToRead);
               // Reset received flag
               _dataReceived = false;
               receive.Step = 4;
               break;

            case 4:
               // Write request string to read the software version
               _com1.Write(rs232Set, 1, 7);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               receive.Step = 5;
               break;

            case 5:
               //Wait for event handler to say data recieved
               if (_dataReceived)
               {
                  receive.Step = 6;
               }
               if (Environment.TickCount > _endTicks)
               {
                  // No response within 1 second
                  receive.Step = 40;
               }
               break;

            case 6:
               // receive data from buffer
               _com1.Read(rs232Buff, 0, _com1.BytesToRead);
               _dataReceived = false;
               // Check if the response back is an 'O' character to say data request was received OKEn
               if (rs232Buff[0] == 79)
               {
                  receive.Step = 7;
               }
               else
               {
                  // error in reading response from the Epos
                  receive.Step = 30;
               }

               break;

            case 7:
               // write acknowledgement
               _com1.Write(rs232Set, 8, 1);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               receive.Step = 8;
               break;

            case 8:
               //Wait for event handler data recieved
               if (_dataReceived)
               {
                  receive.Step = 9;
               }
               if (Environment.TickCount > _endTicks)
               {
                  // No response within 1 second
                  receive.Step = 40;
               }
               break;

            case 9:
               // receive data from the buffer
               _com1.Read(rs232Buff, 0, _com1.BytesToRead);
               // reset data received flag            
               try
               {
                  // interpret the reponse from the controller -> extract the 5th and 6th byte and convert to a word
                  _statusWord = (ushort)((((ushort)(rs232Buff[6] * 256)) + ((ushort)rs232Buff[5]) & 0x417F));
               }
               catch (Exception e)
               {
                  // exception error handling
                  _systemStatus = e.ToString();
                  // Control word could not be interpreted
                  receive.Step = 30;
                  break;
               }
               _dataReceived = false;
               receive.Step = 10;
               break;

            case 10:
               // write acknowledgement (character "O")
               _com1.Write(rs232Set, 8, 1);

               // Report successful finish
               receive.Step = 20;
               break;

            // Operation Finished sucessfully
            case 20:
               receive.Step = 0;
               receive.State = Status.FinishedOk;
               receive.Message = "Operation Finished Sucessfully";
               break;

            // Communications error received
            case 30:
               receive.Step = 0;
               receive.State = Status.Failed;
               receive.Message = "Communication Error Received";
               break;

            // Operation timed out
            case 40:
               receive.Step = 0;
               receive.State = Status.TimedOut;
               receive.Message = "Operation Timed Out";
               break;
         }
         return receive;
      }
      #endregion RequestData

      #region SendData


      private RWSts SendData(byte[] rs232Set, RWSts send)
      {
         // buffer that stores the data received back from the EPOS           

         if (_com1.IsOpen)
         {
            byte[] RS232buffer = new byte[_com1.BytesToRead];
            switch (send.Step)
            {
               // initialise variables used for communication
               case 0:
                  send.Step = 1;
                  send.State = Status.Processing;
                  break;

               case 1:
                  // send the first byte in the array, representing the op code
                  _com1.Write(rs232Set, 0, 1);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  // Go to next step
                  send.Step = 2;
                  break;

               case 2:
                  // Wait for event handler data recieved
                  if (_dataReceived)
                  {
                     send.Step = 3;
                  }
                  if (Environment.TickCount > _endTicks)
                  {
                     // No response within 1 second
                     send.Step = 40;
                  }
                  break;

               case 3:
                  // Read the response from the EPOS                          
                  _com1.Read(RS232buffer, 0, _com1.BytesToRead);
                  _dataReceived = false;
                  send.Step = 4;
                  break;

               case 4:
                  // Write request to read  (bytes 1->11 are part of the request)
                  _com1.Write(rs232Set, 1, 11);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  send.Step = 5;
                  break;

               case 5:
                  // Wait for event handler data recieved
                  if (_dataReceived)
                  {
                     send.Step = 6;
                  }
                  if (Environment.TickCount > _endTicks)
                  {
                     // No response within 1 second
                     send.Step = 40;
                  }
                  break;

               case 6:
                  // Read the response from the EPOS                          
                  _com1.Read(RS232buffer, 0, _com1.BytesToRead);
                  _dataReceived = false;
                  // Check if the response back is an 'O' character to say OPCode was received OKEn
                  if (RS232buffer[0] == 79)
                  {
                     send.Step = 7;
                  }
                  else
                  {
                     // Error was received
                     send.Step = 30;
                  }
                  break;

               case 7:
                  // send the 12th byte in the array, representing an acknowledgement (character 'O')
                  _com1.Write(rs232Set, 12, 1);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  // Go to next step
                  send.Step = 8;
                  break;

               case 8:
                  // Wait for event handler data recieved
                  if (_dataReceived)
                  {
                     send.Step = 9;
                  }
                  if (Environment.TickCount > _endTicks)
                  {
                     // No response within 1 second
                     send.Step = 40;
                  }
                  break;

               case 9:
                  // Read the response from the EPOS                          
                  _com1.Read(RS232buffer, 0, _com1.BytesToRead);
                  _dataReceived = false;
                  send.Step = 10;
                  break;

               case 10:
                  // send the 12th byte in the array, representing an acknowledgement (character 'O')
                  _com1.Write(rs232Set, 12, 1);

                  // Operation finished sucessfully
                  send.Step = 20;
                  break;

               // Operation Finished sucessfully
               case 20:
                  send.Step = 0;
                  send.State = Status.FinishedOk;
                  send.Message = "Operation Finished Sucessfully";
                  break;

               // Communications error received
               case 30:
                  send.Step = 0;
                  send.State = Status.Failed;
                  send.Message = "Communication Error Received";
                  break;

               // Operation timed out
               case 40:
                  send.Step = 0;
                  send.State = Status.TimedOut;
                  send.Message = "Operation Timed Out";
                  break;
            }
         }
         return send;
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
