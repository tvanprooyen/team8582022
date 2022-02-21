// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DefaultDriveCommand;
//import frc.robot.commands.ShootingPIDCmd;
//import frc.robot.commands.ConveyerCmd;
//import frc.robot.commands.ArmPID;

//import frc.robot.subsystems.ArmControl;
//import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DrivetrainSubsystem;
//import frc.robot.subsystems.Shooting;


public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  //private final ArmControl armsubsystem = new ArmControl();
  
  
 // private final Shooting shooter = new Shooting();
 // private final ArmControl armcontrol = new ArmControl();
 // private final Conveyer convey = new Conveyer();
  

  private final Joystick driver1 = new Joystick(0);
 //private final Joystick driver2 = new Joystick(1);

  public RobotContainer() {
    //configures joytsick to drivetrainsubsystem
    //1: up and down 
    //0: left and right
    // 4: rotation
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis( driver1.getRawAxis(1)) * DrivetrainSubsystem.MaxVelocity, 
            () -> -modifyAxis( driver1.getRawAxis(0)) * DrivetrainSubsystem.MaxVelocity,
            () -> -modifyAxis( driver1.getRawAxis(4)) * DrivetrainSubsystem.MaxAngularVelocity 
    ));
   
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
   
   new JoystickButton(driver1,7).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
/*
    //shooter
    //I dont know why but 5500 dosent get it up to full speed 
    new JoystickButton(driver2, 3).whileActiveOnce(new ShootingPIDCmd(shooter, 20000, true) );
    new JoystickButton(driver2, 4).whileActiveOnce(new ShootingPIDCmd(shooter, 1, false) );

    //intake
    new JoystickButton(driver2,200).whileActiveOnce(new ArmPID(armcontrol, 2));

    //conveyer
    //i'm trying my best and have not tested this
    new JoystickButton(driver2, 6).whileActiveOnce(new ConveyerCmd(convey, 0.5));
*/
 
  }

 
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }


  //dead band functions
  private static double deadband(double value, double deadband) {
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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
