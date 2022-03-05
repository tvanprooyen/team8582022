
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.team858.control.ControlMathUtil;
import com.team858.control.Joystick_858;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DefaultDriveCommand;
//import frc.robot.commands.ShootingPIDCmd;
//import frc.robot.commands.ConveyerCmd;
//import frc.robot.commands.ArmPID;
import frc.robot.subsystems.ArmControl;
//import frc.robot.subsystems.ArmControl;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DrivetrainSubsystem;
//import frc.robot.subsystems.Shooting;


public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  //private final ArmControl armsubsystem = new ArmControl();
  
  
  //private final Shooting shooter = new Shooting();
 // private final ArmControl armcontrol = new ArmControl();
  private final Conveyer convey = new Conveyer();
  private final ArmControl armControl = new ArmControl();
 //rivate final DefaultDriveCommand Drivetrain = new DefaultDriveCommand();
  

  private final Joystick driver1 = new Joystick(Constants.Driver1Port);
  //private final Joystick driver2 = new Joystick(1);

  public RobotContainer() {
    //configures joytsick to drivetrainsubsystem
    //1: up and down 
    //0: left and right
    // 4: rotation
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -ControlMathUtil.modifyAxis(driver1.getRawAxis(1), Constants.deadband) * DrivetrainSubsystem.MaxVelocity, 
            () -> -ControlMathUtil.modifyAxis(driver1.getRawAxis(0), Constants.deadband) * DrivetrainSubsystem.MaxVelocity,
            () -> -ControlMathUtil.modifyAxis(m_drivetrainSubsystem.injectedRotation(4,3),Constants.deadband) * DrivetrainSubsystem.MaxAngularVelocity
    ));
   

    // Configure the button bindings
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    //3: Track Target
   
    //reset gyro
   new JoystickButton(driver1,7).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

  //new JoystickButton(driver1,2).whileHeld(new DefaultDriveCommand(m_drivetrainSubsystem, () ->  driver1.getRawAxis(1), () -> driver1.getRawAxis(0), () -> m_drivetrainSubsystem.injectedRotation(4,3)));

 //shooter
  //new JoystickButton(driver1, 1).whileActiveOnce(new ShootingPIDCmd(shooter, 20000, true) );

  //conveyor
  //new JoystickButton(driver1, 2).whileActiveOnce(new ConveyerCmd(convey, -0.2));


   /*
    
   
    //intake
    new JoystickButton(driver2,200).whileActiveOnce(new ArmPID(armcontrol, 2));
 
*/
 
  }

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

 
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}