/*
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;



public class Shooting extends SubsystemBase {


       private final CANSparkMax Shooter = new CANSparkMax(9,MotorType.kBrushless);
       private final RelativeEncoder Shooter_encoder = Shooter.getEncoder();
       private final SparkMaxPIDController ShooterPID = Shooter.getPIDController();
   
    public Shooting(){

    }

    public void setShooter(double setPoint){
        Shooter.set(setPoint);
    }
    
    @Override
    public void periodic(){ 
    }
    


}
*/