namespace MotorControllers
{
   public enum Status
   {
      Idle,
      Processing,
      FinishedOk,
      Failed,
      TimedOut
   }

   // keep track of the Machine's progress status
   public class PrgSts
   {
      /// <summary>
      /// Motor's current state
      /// </summary>
      public Status State { get; set; } = Status.Idle;
      
      /// <summary>
      /// Error code used for debugging purposes
      /// </summary>
      public int Value { get; set; } = 0;
      
      /// <summary>
      /// Error message
      /// </summary>
      public string Message { get; set; } = string.Empty;

      /// <summary>
      /// Keeps track of the motor's step transition
      /// </summary>
      public int Step { get; set; } = 0;
   }
}
