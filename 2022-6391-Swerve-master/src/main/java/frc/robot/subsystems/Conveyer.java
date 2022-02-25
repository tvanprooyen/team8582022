package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

public class Conveyer extends SubsystemBase {  
    /* private DigitalInput sensor0,sensor1,sensor2,sensor3,sensor4;
    private final CANSparkMax conveyer = new CANSparkMax(ConveyorID,MotorType.kBrushless);

    private boolean latch;

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

    public boolean latch(boolean latch) {
        this.latch = false;
        return latch;
    }

    public boolean getLatch() {
        return this.latch;
    }

    public void gate() {
        if( 
            (!getSensor0() && !getSensor1()) ||
            (getLatch() && getSensor1())

        ) latch(false);  return;
        
        if(
            getSensor0() ||
            (getSensor0() && getSensor1())
        ) latch(true); return;
    }

    @Override
    public void periodic(){
        gate();

        if(getLatch()) {
            setSpeed(0.1);
        } else {
            setSpeed(0);
        }
    }
    

    public void setSpeed(double speed){
         conveyer.set(speed);
    } */
}

