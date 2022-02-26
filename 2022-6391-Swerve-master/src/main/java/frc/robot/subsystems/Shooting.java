/* 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import static frc.robot.Constants.*;

public class Shooting extends SubsystemBase {
    private final CANSparkMax Shooter = new CANSparkMax(ShooterID,MotorType.kBrushless);
    private final RelativeEncoder ShooterEncoder = Shooter.getEncoder();
    
    public Shooting(){}

    public CANSparkMax getSpark(){
        return Shooter;
    }

    public void setShooter(double speed){
        //Shooter.set(speed);
    }

    public double getEncoder(){
        return ShooterEncoder.getVelocity();
    }

    
    @Override
    public void periodic(){ 
    }
}


 */