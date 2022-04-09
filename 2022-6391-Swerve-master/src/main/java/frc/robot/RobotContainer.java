
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.team858.control.ControlMathUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.ArmControl;
import frc.robot.subsystems.ClimbingSubsys;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DrivetrainSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final Conveyer convey = new Conveyer();
    private final ArmControl armControl = new ArmControl();
    private final ClimbingSubsys climb = new ClimbingSubsys();
    private final SlewRateLimiter xLimit = new SlewRateLimiter(300);
    private final SlewRateLimiter yLimit = new SlewRateLimiter(300);

    private final SendableChooser<Integer> m_chooser = new SendableChooser<>();

    private final Joystick driver1 = new Joystick(Constants.Driver1Port);
    

    public RobotContainer() {
        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Drive Back and Shoot", 0);
        m_chooser.addOption("Drive to ball, turn, and Shoot", 1);
        m_chooser.addOption("Drive to ball and STOP", 2);
        m_chooser.addOption("Movement Test", 3);
        //m_chooser.addOption("Gyro Test", 3);

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);

        //configures joytsick to drivetrainsubsystem
        //1: up and down 
        //0: left and right
        // 4: rotation

        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrainSubsystem,
                () -> -yLimit.calculate(ControlMathUtil.modifyAxis(driver1.getRawAxis(1), Constants.deadband) * DrivetrainSubsystem.MaxVelocity), 
                () -> -xLimit.calculate(ControlMathUtil.modifyAxis(driver1.getRawAxis(0), Constants.deadband) * DrivetrainSubsystem.MaxVelocity),
                () -> -(ControlMathUtil.modifyAxis(m_drivetrainSubsystem.injectedRotation(4,3),Constants.deadband) * DrivetrainSubsystem.MaxAngularVelocity) * 0.3,
                () -> driver1.getRawButton(10)
        ));
        // Configure the button bindings
        configureButtonBindings();
    }


    private void configureButtonBindings() {
        //reset gyro
        new JoystickButton(driver1,7).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    }

    public Command getAutonomousCommand() {
        return new AutoDrive(m_drivetrainSubsystem, convey, armControl, m_chooser);
    }
}