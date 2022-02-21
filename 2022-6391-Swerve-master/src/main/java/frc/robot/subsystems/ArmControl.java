/*
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import static frc.robot.Constants.*;

public class ArmControl extends SubsystemBase {
    private final CANSparkMax Arm = new CANSparkMax(ArmID,MotorType.kBrushless);
    private final CANSparkMax Armbelt = new CANSparkMax(ArmBeltID,MotorType.kBrushless);

    public ArmControl(){}

    public void MoveArm(double ArmSpeed){
        Arm.set(ArmSpeed);
    }

    public void MoveArmBelt(){
        Armbelt.set(0.7);
    }
    
    public CANSparkMax getMotor(){
        return Arm;
    }
    @Override
    public void periodic(){}
}
*/
