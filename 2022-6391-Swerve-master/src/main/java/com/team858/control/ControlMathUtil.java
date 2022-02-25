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
}
