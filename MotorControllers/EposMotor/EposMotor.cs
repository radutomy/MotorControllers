using System;
using System.IO.Ports;

namespace MotorControllers.EposMotor
{ 
   public class EposMotor : IEposFunctions
   {
      // serial port object with the appropriate settings
      private readonly SerialPort _comPort;
      
      int _endTicks = 0;                           // used to measure time
      ushort _statusWord;                          // stores the last response from a Read request
      bool _isDataReceived = false;                // turns true when SerialData Received event risen        
      string _systemStatus;                        // used for system diagnostics
      bool _isComEventSet = false;

      public EposMotor(ushort portNum)
      {
         if (portNum < 1)
            throw new ArgumentOutOfRangeException(nameof(portNum) + " cannot be less than zero");
         
         _comPort = new SerialPort("COM" + portNum, 9600, Parity.None, 8, StopBits.One);
      }

      // SerialData Received event handler event raised
      private void DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
      {
         _isDataReceived = true;
      }

      // Read/Write states
      private class RwSts : PrgSts
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

      private RwSts ReceiveData(byte[] rs232Set, RwSts receive)
      {
         // dynamicly re-sizeable array based on the amount of bytes in the serial receive buffer
         var rs232Buff = new byte[_comPort.BytesToRead];

         switch (receive.Step)
         {
            // initialise variables used for communication
            case 0:
               receive.State = Status.Processing;
               receive.Step = 1;
               break;

            case 1:
               //send 1st byte to EPOS2, this is the OPCode (0x10 = Read, 0X11 = Write)
               _comPort.Write(rs232Set, 0, 1);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               // Go to next step 
               receive.Step = 2;
               break;

            case 2:
               // Wait for event handler data recieved
               if (_isDataReceived)
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
               _comPort.Read(rs232Buff, 0, _comPort.BytesToRead);
               // Reset received flag
               _isDataReceived = false;
               receive.Step = 4;
               break;

            case 4:
               // Write request string to read the software version
               _comPort.Write(rs232Set, 1, 7);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               receive.Step = 5;
               break;

            case 5:
               //Wait for event handler to say data recieved
               if (_isDataReceived)
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
               _comPort.Read(rs232Buff, 0, _comPort.BytesToRead);
               _isDataReceived = false;
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
               _comPort.Write(rs232Set, 8, 1);
               // Store curent ticks + 1 second for timeout on recieve
               _endTicks = Environment.TickCount + 1000;
               receive.Step = 8;
               break;

            case 8:
               //Wait for event handler data recieved
               if (_isDataReceived)
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
               _comPort.Read(rs232Buff, 0, _comPort.BytesToRead);
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
               _isDataReceived = false;
               receive.Step = 10;
               break;

            case 10:
               // write acknowledgement (character "O")
               _comPort.Write(rs232Set, 8, 1);

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

      /*  Generic format for writing a value to the motor
     *  This function takes the pased byte aray containing a set of instructions that writes a value to the drive
     *  The communications protocol in the drive requires the parts of the message to be sent in a specific order.
     *  Each time a message is sent the drive responds with a code (either 'O' or 'F') identifying OKEn or Failed
     *  The order of the message structure is as follows:
     *      Step 1 = Send op code byte                                       -> stored in byte 0 of the byte array
     *      Step 2 = Wait for received event triggered, 
     *               if not recieved in 1 second exit under fault 
     *      Step 3 = Check for "O" / 'F' response
     *      Step 4 = Send the write value request                            -> stored in bytes 1-11 of array (See FormatWriteMessage for byte definition)
     *      Step 5 = Wait for received event triggered;             
     *               if not recieved in 1 second exit under fault 
     *      Step 6 = Check for "O" / 'F' response
     *      Step 7 = Send "O" for acknowledgement                           -> stored in byte 12 of the byte array
     *      Step 8 = Wait for the received event triggered
     *               if not received in 1 second exit under fault
     *      Step 9 = Receive a response from the Epos
     *      Step 10 = Send an "O" for acknowledgement                      -> stored in byte 12 of the byte array
     *      --------
     *      Step 20 = Data sent correctly
     *      Step 30 = Communication failure with the Epos 
     *      Step 40 = Time Out in receiving a response from the */
      private RwSts SendData(byte[] rs232Set, RwSts send)
      {
         // buffer that stores the data received back from the EPOS           

         if (_comPort.IsOpen)
         {
            byte[] RS232buffer = new byte[_comPort.BytesToRead];
            switch (send.Step)
            {
               // initialise variables used for communication
               case 0:
                  send.Step = 1;
                  send.State = Status.Processing;
                  break;

               case 1:
                  // send the first byte in the array, representing the op code
                  _comPort.Write(rs232Set, 0, 1);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  // Go to next step
                  send.Step = 2;
                  break;

               case 2:
                  // Wait for event handler data recieved
                  if (_isDataReceived)
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
                  _comPort.Read(RS232buffer, 0, _comPort.BytesToRead);
                  _isDataReceived = false;
                  send.Step = 4;
                  break;

               case 4:
                  // Write request to read  (bytes 1->11 are part of the request)
                  _comPort.Write(rs232Set, 1, 11);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  send.Step = 5;
                  break;

               case 5:
                  // Wait for event handler data recieved
                  if (_isDataReceived)
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
                  _comPort.Read(RS232buffer, 0, _comPort.BytesToRead);
                  _isDataReceived = false;
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
                  _comPort.Write(rs232Set, 12, 1);
                  // Store curent ticks + 1 second for timeout on recieve
                  _endTicks = Environment.TickCount + 1000;
                  // Go to next step
                  send.Step = 8;
                  break;

               case 8:
                  // Wait for event handler data recieved
                  if (_isDataReceived)
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
                  _comPort.Read(RS232buffer, 0, _comPort.BytesToRead);
                  _isDataReceived = false;
                  send.Step = 10;
                  break;

               case 10:
                  // send the 12th byte in the array, representing an acknowledgement (character 'O')
                  _comPort.Write(rs232Set, 12, 1);

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

      /* 
       * The function performs the following:
       * Reads the status of the Epos 
       * At any time, the motor can be
       *      Switch On Disable
       *      Ready To Switch On
       *      SwitchedOn
       *      Operation Enabled      
       *      Fault Mode
       *      QuickStop Active
       *      
       * The purpose of the EnableEposFunction function is to get the motor in the Operation Enabled state
       * In order to do that the motor performs the following steps
       *      Reads the status
       *      If the Motor is in Fault Mode, reset the Fault 
       *      If QuickStop is enabled, turn QuickStop off 
       *      Read the status
       *      If the motor is Switch On Disabled, perform a "ShutDown" function (note: this is not a mistake, the terminilogy is used in the manual)
       *      Read the status
       *      If the motor is Ready to Switch On, or SwitchedOn then perform a 'SwitchOnAndEnableVoltage' command 
       *      The motor is now Enabled and ready to receive commands
       *      
       *      Set the operation mode to Velocity (OpMode = -2)
       *      Set the Maximum Velocity to ...
       *      Set the Maximum Acceleration to...
       *      
       * Step numbers and their function:
       *      0. Initialise variables used for communication
       *         Attempty to open COM port; if the COM port is busy, report InitFail (jump to Step 30)
       *         Attach serial event handler
       *      1. Read Epos Status Word and decode Epos State
       *         Based on the status, perform the following operations; see the SDS flowchart page 36
       *      2. Send ShutDown command
       *      3. Send SwitchOnAndEnVoltage command
       *      4. Send SetOpNo command
       *      5. Send SetMaxVelocity command
       *      6. Send SetMaxAcceleration command; report InitDone (jump to Step 20)
       *      7. Send DisableVoltage command
       *      8. Send ResetFault command
       *      20. Report that the Epos has been Enabled successfully
       *      30. Report that the Epos Enable failed
       *      
       * The index values used for Writing/Reading a value in the Epos (See SDS, page 28)
       *      StatusWord     - 0x6041
       *      ControlWord    - 0x6040
       *      SetOpNumber    - 0x6060
       *      SetMaxVelocity - 0x607F
       *      SetMaxAcc      - 0x60C5
       */

      public PrgSts Enable()
      {
         var initDone = new PrgSts();
         var readWordSts = new RwSts();
         var shutDownSts = new RwSts();
         var resVolSts = new RwSts();
         var setVelSts = new RwSts();
         var swOnSts = new RwSts();
         var setOpSts = new RwSts();
         var setAccSts = new RwSts();
         var disVolSts = new RwSts();

         while (initDone.State == Status.Processing)
         {
            switch (initDone.Step)
            {
               // initialise variables
               case 0:
                  // data received event handler for COM1
                  if (!_isComEventSet)
                  {
                     _isComEventSet = true;
                     _comPort.DataReceived += new SerialDataReceivedEventHandler(DataReceived);
                  }
                  try
                  {
                     // Open the port for communications
                     _comPort.Open();
                     initDone.State = Status.Processing;
                     initDone.Step = 1;
                  }
                  catch (Exception e)
                  {
                     initDone.Step = 30;
                     initDone.Message = "EnableEpos Port Failed to Open";
                  }
                  break;

               /* Decode Epos Status
                * Request the Epos the controlword, by using a formatted array (See FormatReadMessage function)
                * When the ControlWord has been received, decode it and return the state
                */
               case 1:
                  // Read the StatusWord
                  readWordSts = ReceiveData(FormatReadMessage(0x6041, 0x00), readWordSts);

                  // RWState is 2 when Reading the StatusWord was successful
                  if (readWordSts.State == Status.FinishedOk)
                  {
                     // Decode the Status Word
                     // The flowchart on how StatusWord is interpreted can be found in the SDS, page 36
                     switch (_statusWord)
                     {
                        // Epos is Disabled - Perform a 'ShutDown' command
                        case 0x140:
                           initDone.Step = 2;
                           break;

                        // Epos is Ready - Perform a 'SwitchOn and EnableVoltage' command
                        case 0x121:
                           initDone.Step = 3;
                           initDone.Message = "1) Epos is Ready - Perform a 'SwitchOn and EnableVoltage' command";
                           break;

                        // Epos is Switched On - Perform a 'SwichOnAndEnableVoltage' command 
                        case 0x123:
                           initDone.Step = 3;
                           break;

                        // Epos is Enabled - Perform a 'SetOperationNumber' command
                        case 0x137:
                           initDone.Step = 4;
                           break;

                        // QuickStop is on - Reset QuickStop by performing a DisableVoltage command
                        case 0x117:
                           initDone.Step = 7;
                           break;

                        // Epos is in Fault Mode - Perform a 'ResetFault' command
                        case 0x108:
                           initDone.Step = 8;
                           initDone.Message = "1) Epos is in Fault Mode - Perform a 'ResetFault' command";
                           break;
                     }
                  }

                  // if the Read Function timed out or a communication error occured, report that the initialisation failed
                  else if (readWordSts.State == Status.Failed || readWordSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Reading the StatusWord";
                  }
                  break;

               // Shutdown command
               case 2:
                  shutDownSts = SendData(FormatWriteMessage(0x6040, 0x06), shutDownSts);

                  if (shutDownSts.State == Status.FinishedOk)
                  {
                     initDone.Step = 3;
                  }
                  else if (shutDownSts.State == Status.Failed || shutDownSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Writing the ShutDown command";
                  }
                  break;

               // Switch On and Enable Voltage command
               case 3:
                  swOnSts = SendData(FormatWriteMessage(0x6040, 0x0F), swOnSts);

                  if (swOnSts.State == Status.FinishedOk)
                  {
                     initDone.Step = 4;
                  }
                  else if (swOnSts.State == Status.Failed || swOnSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Writing the SwitchOnAndEnableVoltage command";
                  }
                  break;

               // Set Operation Number in Epos
               case 4:
                  setOpSts = SendData(FormatWriteMessage(0x6060, -2), setOpSts);

                  if (setOpSts.State == Status.FinishedOk)
                  {
                     initDone.Step = 5;
                  }
                  else if (setOpSts.State == Status.Failed || setOpSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Writing the SetOperationNumber command";
                  }
                  break;

               // Set Maximum Velocity command
               case 5:
                  setVelSts = SendData(FormatWriteMessage(0x607F, 5000), setVelSts);

                  if (setVelSts.State == Status.FinishedOk)
                  {
                     initDone.Step = 6;
                  }
                  else if (setVelSts.State == Status.Failed || setVelSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Writing the SetMaximumVelocity command";
                  }
                  break;

               // Set Maximum Acceleration command
               // Value changed from  4000 to 1000 by SJH 9th February 2015
               case 6:
                  setAccSts = SendData(FormatWriteMessage(0x60C5, 1000), setAccSts);

                  if (setAccSts.State == Status.FinishedOk)
                  {
                     initDone.Step = 20;
                  }
                  else if (setAccSts.State == Status.Failed || setAccSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Writing the SetMaximumAcceleration command";
                  }
                  break;

               // DisableVoltage command
               case 7:
                  disVolSts = SendData(FormatWriteMessage(0x6040, 0x00), disVolSts);

                  if (disVolSts.State == Status.FinishedOk)
                  {
                     initDone.Step = 1;
                  }
                  else if (disVolSts.State == Status.Failed || disVolSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Writing the DisableVoltage command"; 
                  }
                  break;

               // Reset Voltage command
               case 8:
                  resVolSts = SendData(FormatWriteMessage(0x6040, 0x80), resVolSts);

                  if (resVolSts.State == Status.FinishedOk)
                  {
                     initDone.Step = 1;
                  }
                  else if (resVolSts.State == Status.Failed || resVolSts.State == Status.TimedOut)
                  {
                     initDone.Step = 30;
                     initDone.Message = "Error Writing the ResetFault command";
                  }
                  break;

               // Report Initialisation sucessful
               case 20:
                  initDone.Step = 0;
                  initDone.State = Status.FinishedOk;
                  initDone.Message = "Epos is sucessfully initialised";
                  // close com port when done
                  _comPort.Close();
                  break;

               // report Initialisation Failed
               case 30:
                  initDone.Step = 0;
                  initDone.State = Status.Failed;
                  _comPort.Close();
                  break;
            }
         }
         return initDone;
      }

      /*  DisableEposDrive disables the Epos by writing the ControlWord
       *  Step numbers and their function:
       *      0. Initialise variables used for communication
       *         Attempty to open COM port; if the COM port is busy, report InitFail (jump to Step 30)
       *         Attach serial event handler
       *      1. Write the a disable command via the ControlWord
       *      20. Report that the Epos has been Disabled successfully
       *      30. Report that the Epos Disable command failed
       */
      private static RwSts _disVoltSts = new RwSts();
      public PrgSts Disable()
      {
         var disableDone = new PrgSts();
         while (disableDone.State == Status.Processing)
         {
            switch (disableDone.Step)
            {
               case 0:
                  // data received event handler for COM1
                  _comPort.DataReceived += new SerialDataReceivedEventHandler(DataReceived);
                  try
                  {
                     // Open the port for communications
                     _comPort.Open();
                     disableDone.State = Status.Processing;
                     disableDone.Step = 1;
                  }
                  catch (Exception e)
                  {
                     disableDone.Step = 30;
                     disableDone.Message = "DisableEpos Port Failed to Open";
                  }
                  break;

               // Disable Epos command
               case 1:

                  _disVoltSts = SendData(FormatWriteMessage(0x6040, 0x00), _disVoltSts);

                  if (_disVoltSts.State == Status.FinishedOk)
                  {
                     disableDone.Step = 20;
                  }
                  else if (_disVoltSts.State == Status.Failed || _disVoltSts.State == Status.TimedOut)
                  {
                     disableDone.Step = 30;
                     disableDone.Message = "Error Writing the DisableEpos command";
                  }
                  break;

               // Report Disable Epos successful
               case 20:
                  disableDone.Step = 0;
                  disableDone.State = Status.FinishedOk;
                  disableDone.Message = "Epos is sucessfully Disabled";
                  // close com port when done
                  _comPort.Close();
                  break;

               // Report Disable Epos failed
               case 30:
                  disableDone.Step = 0;
                  disableDone.State = Status.Failed;
                  _comPort.Close();
                  break;
            }
         }
         return disableDone;
      }
      
      /*  SetSpeed sets the Epos CalculatedSpeed by writing the Velocity Mode RPM
       *  The function takes a the CalculatedSpeed in mm/min as a parameter and converts it into the RPM
       *  The resulting RPM is used to writing the Velocity Mode RPM setting
       *  Step numbers and their function:
       *      0. Initialise variables used for communication
       *         Attempty to open COM port; if the COM port is busy, report InitFail (jump to Step 30)
       *         Attach serial event handler
       *      1. Write the CalculatedSpeed via the Velocity Mode RPM
       *      20. Report that the CalculatedSpeed has been set successfully
       *      30. Report that the CalculatedSpeed setting command has failed
       */
      public PrgSts SetSpeed(double speed)
      {
         var prgSetSpeed = new PrgSts();
         var setSpdSts = new RwSts();
         // calculate the RPM as a function of speed (speed is in mm/s)
         Int32 rpm = (Int32) (((speed * 81) / (Math.PI * 84)) * 60);

         while (prgSetSpeed.State == Status.Processing)
         {
            switch (prgSetSpeed.Step)
            {
               case 0:

                  // data received event handler for COM1
                  _comPort.DataReceived += new SerialDataReceivedEventHandler(DataReceived);
                  try
                  {
                     // Open the port for communications  
                     _comPort.Open();
                     prgSetSpeed.State = Status.Processing;
                     prgSetSpeed.Step = 1;
                  }
                  catch (Exception e)
                  {
                     prgSetSpeed.Step = 30;
                     prgSetSpeed.Message = "SetSpeed Port Failed to Open/n" + e.Message;
                  }
                  break;

               // Set speed Epos command
               case 1:

                  setSpdSts = SendData(FormatWriteMessage(0x206B, rpm), setSpdSts);
                  if (setSpdSts.State == Status.FinishedOk)
                  {
                     prgSetSpeed.Step = 20;
                  }
                  else if (setSpdSts.State == Status.Failed || setSpdSts.State == Status.TimedOut)
                  {
                     prgSetSpeed.Step = 30;
                     prgSetSpeed.Message = "Error Writing the SetSpeed command";
                  }
                  break;

               // Report Disable Epos successful
               case 20:
                  prgSetSpeed.Step = 0;
                  prgSetSpeed.State = Status.FinishedOk;
                  prgSetSpeed.Message = "Motor speed sucessfully set";
                  // close com port when done
                  _comPort.Close();
                  break;

               // Report Disable Epos failed
               case 30:
                  prgSetSpeed.Step = 0;
                  prgSetSpeed.State = Status.Failed;
                  _comPort.Close();
                  break;
            }
         }
         return prgSetSpeed;
      }
   }
}
