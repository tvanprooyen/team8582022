/*
package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class ShootingPIDCmd extends CommandBase {
    private final Shooting shooter;
    private SparkMaxPIDController shooterPID;
    private final double speed;
    private final boolean mode;

    public ShootingPIDCmd(Shooting shooter, double speed, boolean mode){
        this.shooter = shooter;
        this.speed = speed;
        this.mode = mode;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooterPID = shooter.getPID();
        shooterPID.setP(6e-5);shooterPID.setI(0);shooterPID.setD(0);shooterPID.setIZone(0);shooterPID.setFF(0.000015);shooterPID.setOutputRange(-1, 1);
    }

    @Override
    public void execute(){
        if(mode){
            shooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        } else {
            shooter.setShooter(speed);
        }
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
*/