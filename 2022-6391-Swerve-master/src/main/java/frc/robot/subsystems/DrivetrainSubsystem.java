
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.team858.control.Joystick_858;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

public class DrivetrainSubsystem extends SubsystemBase {
  
    

    //Voltage of the battery
    public static final double Voltage = 12.0;
  
    //Max velocity of the wheels in meters per seconds
    public static final double MaxVelocity =   
    5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
 
    //Max angular velocity in radians per seconds
    public static final double MaxAngularVelocity = MaxVelocity /
    Math.hypot(TrackWidth / 2.0, WheelBase / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
        new Translation2d(TrackWidth / 2.0, WheelBase / 2.0),
    // Front right
        new Translation2d(TrackWidth / 2.0, -WheelBase / 2.0),
    // Back left
        new Translation2d(-TrackWidth / 2.0, WheelBase / 2.0),
    // Back right
        new Translation2d(-TrackWidth / 2.0, -WheelBase / 2.0)
  );

    //Set up for navx gyro over mxp
    private final AHRS m_navx = new AHRS(Port.kOnboard, (byte) 200);
    

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;


    private final Joystick driver1 = new Joystick(Driver1Port);

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {

        SmartDashboard.putNumber("Kp", -0.05f);
        SmartDashboard.putNumber("min", 0.05f);
     
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk4SwerveModuleHelper.createNeo(tab.getLayout("fl", BuiltInLayouts.kList), Mk4SwerveModuleHelper.GearRatio.L2, FLdriveID, FLsteerID, FLencoderID, FLoffset);
        m_frontRightModule = Mk4SwerveModuleHelper.createNeo(tab.getLayout("fr", BuiltInLayouts.kList), Mk4SwerveModuleHelper.GearRatio.L2, FRdriveID, FRsteerID, FRencoderID, FRoffset);
        m_backLeftModule = Mk4SwerveModuleHelper.createNeo(tab.getLayout("bl", BuiltInLayouts.kList), Mk4SwerveModuleHelper.GearRatio.L2, BLdriveID, BLsteerID, BLencoderID, BLoffset);
        m_backRightModule = Mk4SwerveModuleHelper.createNeo(tab.getLayout("br", BuiltInLayouts.kList), Mk4SwerveModuleHelper.GearRatio.L2, BRdriveID, BRsteerID, BRencoderID, BRoffset);
    }

    //zeroes the gyro
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public double getLimelightTX(){
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        return tx;
    }

    //gyro rotation
    public Rotation2d getGyroscopeRotation() {
        if (m_navx.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    
    public double injectedRotation(int m_axis, int m_trackButton) {
        //Constants
        double getSteerConstant = 0;
        double Kp = SmartDashboard.getNumber("Kp",-0.05f);
        double min_command = SmartDashboard.getNumber("min",0.05f);
         
        //Target
        double tx = getLimelightTX();
         
        //PID
        double heading_error = -tx;
        double steering_adjust = 0.0f;
        if (tx > 1.0){
                steering_adjust = Kp*heading_error - min_command;
        }else if (tx < 1.0){
                steering_adjust = Kp*heading_error + min_command;
        }

        //Post to Dashboard
        SmartDashboard.putNumber("tv", getLimelightTX());

        //Default to JoyStick
        double Rotation;

        //Activly adjust twords target
        if(driver1.getRawButton(m_trackButton)){
            Rotation = steering_adjust;
        } else {
            Rotation = driver1.getRawAxis(m_axis);
        }

        return Rotation;
    }


    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MaxVelocity);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MaxVelocity * Voltage, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MaxVelocity * Voltage, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MaxVelocity * Voltage, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MaxVelocity * Voltage, states[3].angle.getRadians());

        /* 
        //Depreciated Code(Incase of new doesn't work)
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MaxVelocity);

        double getSteerConstant = 0;
        double Kp = -0.5f;
        double min_command = 0.05f;
         
         
         double tx = getLimelightTX();
         
        
        double heading_error = -tx;
        double steering_adjust = 0.0f;
        if (tx > 1.0){
                steering_adjust = Kp*heading_error - min_command;
        }else if (tx < 1.0){
                steering_adjust = Kp*heading_error + min_command;
        }

        SmartDashboard.putNumber("tv", getLimelightTX());

        double Rotation0 = states[0].speedMetersPerSecond / MaxVelocity * Voltage;
        double Rotation1 = states[1].speedMetersPerSecond / MaxVelocity * Voltage;
        double Rotation2 = states[2].speedMetersPerSecond / MaxVelocity * Voltage;
        double Rotation3 = states[3].speedMetersPerSecond / MaxVelocity * Voltage;

        

        if(driver1.getRawButton(3)){
            Rotation0 += steering_adjust;
            Rotation1 -= steering_adjust;
            Rotation2 += steering_adjust;
            Rotation3 -= steering_adjust;
        }

        m_frontLeftModule.set(Rotation0, states[0].angle.getRadians());
        m_frontRightModule.set(Rotation1, states[1].angle.getRadians());
        m_backLeftModule.set(Rotation2, states[2].angle.getRadians());
        m_backRightModule.set(Rotation3, states[3].angle.getRadians()); */
    }
}
