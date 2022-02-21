/*
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

public class Conveyer extends SubsystemBase {  
    private DigitalInput sensor0,sensor1,sensor2,sensor3,sensor4;
    private final CANSparkMax conveyer = new CANSparkMax(ConveyorID,MotorType.kBrushless);

    public Conveyer(){}

    public boolean getSensor0(){
        return sensor0.get();
    }

    public boolean getSensor1(){
        return sensor1.get();
    }

    public boolean getSensor2(){
        return sensor2.get();
    }

    public boolean getSensor3(){
        return sensor3.get();
    }

    public boolean getSensor4(){
        return sensor4.get();
    }

    @Override
    public void periodic(){}

        public void setSpeed(double speed){
         conveyer.set(speed);
    }
}
*/
