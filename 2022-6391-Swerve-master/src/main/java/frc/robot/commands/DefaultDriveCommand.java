
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final boolean mode;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier, boolean mode) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.mode = mode;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
       
        if(mode){
        //normal drive 
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
        } else {
        //steer with limelight

         double getSteerConstant = 0;
         float Kp = -0.1f;
         float min_command = 0.05f;
         
         
         double tx = m_drivetrainSubsystem.getLimelightTX();
         
        
                 float heading_error = -tx;
                 float steering_adjust = 0.0f;
                 if (tx > 1.0){
                         steering_adjust = Kp*heading_error - min_command;
                 }else if (tx < 1.0){
                         steering_adjust = Kp*heading_error + min_command;
                 }
                     
            



            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0.0,
                       0.0,
                        steering_adjust,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                );
        }

        SmartDashboard.putNumber("tv",m_drivetrainSubsystem.getLimelight());
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
