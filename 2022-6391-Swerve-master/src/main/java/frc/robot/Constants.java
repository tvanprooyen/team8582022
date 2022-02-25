package frc.robot;

import com.team858.control.Joystick_858;

public final class Constants {
    //Left to right between wheels in meters
    public static final double TrackWidth = 0.5762625;
    //front to back between wheels in meters
    public static final double WheelBase = 0.5762625;

    //Front Left 
    public static final int FLdriveID = 2; 
    public static final int FLsteerID = 3;
    public static final int FLencoderID = 26; 
    public static final double FLoffset = -Math.toRadians(171.74);

    //Front Right
    public static final int FRdriveID = 6;
    public static final int FRsteerID = 7; 
    public static final int FRencoderID = 29;
    public static final double FRoffset = -Math.toRadians( 232.38);

    //Back Left
    public static final int BLdriveID = 4;
    public static final int BLsteerID = 5; 
    public static final int BLencoderID = 27;
    public static final double BLoffset = -Math.toRadians(317.81);

    //Back Right
    public static final int BRdriveID = 8;
    public static final int BRsteerID = 9;
    public static final int BRencoderID = 28;
    public static final double BRoffset = -Math.toRadians(127.0); 

    //Controls
    public static final int Abutton = 0;

    //SparkMax ids
    public static final int ShooterID = 7; //place holders for now
    public static final int ArmID = 201;
    public static final int ArmBeltID = 202;
    public static final int ConveyorID = 203;

    //Slew Rate Limiter
    public static final double changeRate = 0.5;
   
    //Dead Band
    public static final double deadband = 0.2;

    //Modify
    public static final boolean modify = true;
}
