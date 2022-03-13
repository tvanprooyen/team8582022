/* 
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Auto extends SubsystemBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final PIDController pid = new PIDController(0.004, 0, 0);

    private final Joystick Driver = new Joystick(0);

    //Speed Enum
    public enum AutoMode {
        NONE(0), AutoMode1(1), AutoMode2(2), AutoMode3(3);
    
        private final int value;
        private AutoMode(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    //public void atuo


    Auto() {

    }

    public void dashboard() {
     
    }

    public double getEncoderAverage(){
        double average = m_drivetrainSubsystem.getAverageEncoder();
        return average;
    }

    public Rotation2d getGyroRotation(){
        Rotation2d rotation = m_drivetrainSubsystem.getGyroscopeRotation();
        return rotation;
    }


    //drive off tarmac
    public void strategy1(){
       //move back and left;
       double OrthogonalXSpeed = 0;
       double OrthogonalYSpeed = 0;
       double DiagonalSpeed = 0;

    
       
     

     

       
    
    }

    //drive off tarmac and shoot
    public void strategy2(){
      
    }

    //drive off, pick up ball, and shoot
    public void strategy3(){
       
    }




    @Override
    public void periodic() { */
        /*
        AutoMode autoMode = AutoMode.valueOf(SmartDashboard.getNumber("Auto Mode", 0));
        
        switch(autoMode){
            case 1: 
                strategy1();
                break;
            case 2: 
                strategy2();
                break;
            case 3: 
                strategy3();
                break;
            default: //bruh;
        }

      */
/*       if(Driver.getRawButton(1)){

        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            0.1,
            0.1,
            0.01
    ));
    }
        dashboard();
    }
    
} */

 