package com.team858.control;

public class ControlMathUtil {
    //dead band functions
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
    
      public static double modifyAxis(double value, double deadband) {
        // Deadband
        value = deadband(value, deadband);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }

       /**
       * Allows user to manually change a PID setpoint
       *
       * @param direction Direction UP(plus), DOWN(minus), NONE(nothing happens)
       * @param rateOfChange How much is added to the old setpoint
       * @return The next setpoint
       */
      public double chaser(enums.Direction direction, double rateOfChange, double chaser) {
        if(direction == enums.Direction.UP) {
            chaser += rateOfChange;
        } else if (direction == enums.Direction.DOWN) {
            chaser -= rateOfChange;
        }

        return chaser;
    }
}
