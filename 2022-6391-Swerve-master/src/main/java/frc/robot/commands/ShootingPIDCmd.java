
package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingPIDCmd extends CommandBase {
    private final Shooting shooter;
    private final double speed;
    private SparkMaxPIDController ShooterPID;
    private final boolean mode;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public ShootingPIDCmd(Shooting shooter, double speed, boolean mode){
        this.shooter = shooter;
        this.speed = speed;
        this.ShooterPID = shooter.getSpark().getPIDController();
        this.mode = mode;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        kP = 6e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
    
        // set PID coefficients
        ShooterPID.setP(kP);
        ShooterPID.setI(kI);
        ShooterPID.setD(kD);
        ShooterPID.setIZone(kIz);
        ShooterPID.setFF(kFF);
        ShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void execute(){
        
        if( mode){
       ShooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
      
        } else {
            shooter.setShooter(speed);
        }
        SmartDashboard.putNumber("Speed",shooter.getEncoder());
    }

    @Override
    public void end(boolean interrupted){
        shooter.setShooter(0);
    }

    @Override
    public boolean isFinished(){
        return false;            
    }
}
