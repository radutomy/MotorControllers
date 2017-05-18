namespace MotorControllers
{
   public interface IMotorBasicFunctions
   {
      /// <summary>
      /// Enable the motor's functions
      /// </summary>
      /// <returns></returns>
      PrgSts Enable();

      /// <summary>
      /// Disable the motor's functions
      /// </summary>
      /// <returns></returns>
      PrgSts Disable();

      /// <summary>
      /// Sets the motor's speed to the given value and starts (runs) the motor
      /// </summary>
      /// <param name="speed">Speed in mm/min</param>
      /// <returns></returns>
      PrgSts SetSpeed(double speed);
   }
}
