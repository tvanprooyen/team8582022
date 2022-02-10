// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. Eric was here

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import com.kauailabs.navx.*;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Joystick Driver1, Driver2;
  private CANSparkMax     IntakePositioner, Shooter, Arm;
  private RelativeEncoder IntakePositionerEncoder, ShooterEncoder, ArmEncoder;
  private DigitalInput    sensor1, sensor2, sensor3, sensor4, sensor5;
  private SparkMaxPIDController ShooterPID, FieldOrientationPID, DriveSpeedPID, SteeringPID, ArmPID;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private double kP2, kI2, kD2, kIz2, kFF2, kMaxOutput2, kMinOutput2;
  private double rotations = 0;
  private DrivetrainSubsystem swerve;
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    //camera
    CameraServer.startAutomaticCapture();

    //Joysticks
    Driver1 = new Joystick(0);
    Driver2 = new Joystick(1);

    //Intake System
    IntakePositioner = new CANSparkMax( 10, MotorType.kBrushless);
    IntakePositionerEncoder = IntakePositioner.getEncoder();
    
    //Shooter System
    Shooter = new CANSparkMax( 9, MotorType.kBrushless);
    ShooterEncoder = Shooter.getEncoder();

    //Arm
    Arm = new CANSparkMax( 20, MotorType.kBrushless);
    ArmEncoder = Arm.getEncoder();

    //Motion Sensors
    sensor1 = new DigitalInput(0);
    sensor2 = new DigitalInput(1);
    sensor3 = new DigitalInput(2);
    sensor4 = new DigitalInput(3);
    sensor5 = new DigitalInput(4);

    //PID Systems
    ShooterPID = Shooter.getPIDController();
   
    kP = 5e-5;kI = 1e-6;kD = 0; kIz = 0; kFF = 0.000156; kMaxOutput = 1; kMinOutput = -1;maxRPM = 5700;
    maxVel = 2000; maxAcc = 1500;
    ShooterPID.setP(kP);ShooterPID.setI(kI);ShooterPID.setD(kD);
    ShooterPID.setIZone(kIz);ShooterPID.setFF(kFF);
    ShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    ShooterPID.setSmartMotionMaxVelocity(maxVel, 0);
    ShooterPID.setSmartMotionMinOutputVelocity(minVel, 0);
    ShooterPID.setSmartMotionMaxAccel(maxAcc, 0);
    ShooterPID.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

    ArmPID = Arm.getPIDController();

    kP2 = 0.1; kI2 = 1e-4;kD2 = 1; kIz2 = 0; kFF2 = 0; kMaxOutput2 = 1; kMinOutput2 = -1;
    ArmPID.setP(kP2);ArmPID.setI(kI2);ArmPID.setD(kD2);ArmPID.setIZone(kIz2);ArmPID.setFF(kFF2);
    ArmPID.setOutputRange(kMinOutput2, kMaxOutput2);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {




   

    if( Driver2.getRawButton(1)){
        IntakePositioner.set(1);
    }

    if( Driver2.getRawButton(2)){
        Shooter.set(0.3);
    }

  

    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
}


public class b extends RobotBase {
  private void Shooting() {


     double setPoint = 9000000;            
    ShooterPID.setReference(setPoint, ControlType.kVelocity);
   
    if( Driver2.getRawButton(0)){
      if( ShooterEncoder.getVelocity() > setPoint){
        Shooter.set(1);
      }
    }


  }
  private void PickingUp(){
    ArmPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    if( Driver2.getRawButton(1)){
       rotations = 4;
    } else if(Driver2.getRawButton(2)){
       rotations = 7;
    } 

  }
  private void Drivweee(){
       //its just drive fast boi

  }
  private void Climbing(){
      //in your fdreams
  
  }
  static int Limelight(){
       



}

}
  