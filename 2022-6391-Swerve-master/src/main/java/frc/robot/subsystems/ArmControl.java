
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax Intake = new CANSparkMax(10,MotorType.kBrushless);

    public void IntakeSubsytem() {

    }

    public void setIntake(double b){
        Intake.set(b);
    }
    
    @Override
    public void periodic(){ 
    }

}
