
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import static frc.robot.Constants.*;

public class ArmControl extends SubsystemBase {
    private final CANSparkMax Arm = new CANSparkMax(ArmID,MotorType.kBrushless);
    private final CANSparkMax Armbelt = new CANSparkMax(ArmBeltID,MotorType.kBrushless);
    
    //Magnetic Limit Switches
    private DigitalInput LimitSwitchTop = new DigitalInput(ArmMagneticLimitSwitchTop);
    //private DigitalInput LimitSwitchBottom = new DigitalInput(ArmMagneticLimitSwitchBottom);

    private final Encoder armEncoder = new Encoder(ArmEncoderA, ArmEncoderB, ArmEncoderI, false);
    private final Joystick driver2 = new Joystick(Driver2Port);
    private final PIDController pid = new PIDController(0.004, 0, 0);

    private boolean a_start = false;

    private double a_chaser;

    //Directon Enum
    public enum Direction {
        NONE(0), UP(1), RIGHT(2), DOWN(3), LEFT(4);
    
        private final int value;
        private Direction(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    public ArmControl(){

        //Default Start Location
        this.a_chaser = -40;
    }
/*
    public void MoveArm(double ArmSpeed){
        Arm.set(ArmSpeed);
    }

    public void MoveArmBelt(){
        Armbelt.set(0.7);
    }
    
    public CANSparkMax getMotor(){
        return Arm;
    }*/

    /**
   * Allows user to manually change a PID setpoint
   *
   * @param direction Direction UP(plus), DOWN(minus), NONE(nothing happens)
   * @param rateOfChange How much is added to the old setpoint
   * @return The next setpoint
   */
    private double chaser(Direction direction, double rateOfChange) {
        if(direction == Direction.UP) {
            return this.a_chaser += rateOfChange;
        } else if (direction == Direction.DOWN) {
            return this.a_chaser -= rateOfChange;
        }

        return this.a_chaser;
    }

    public double home() {
        if(LimitSwitchTop.get()) {
            armEncoder.reset();
            a_start = true;
            return 0;
        } else {
            return 0.1; // TODO May need to change value. Should drive up twords the top, at a slow speed
        }
    }


    public void dashboard() {
        //Send Vaules to Dashboard
        //SmartDashboard.putNumber("Arm Encoder", armEncoder.get());
    }


    @Override
    public void periodic(){
        double armSpeed = 0;
        double armBeltSpeed = 0;
        double rateOfChange = 0.1;
        double setpoint = this.a_chaser;


        if(driver2.getRawButton(7)) {
            armEncoder.reset(); //TODO Remove when limit switch added
            a_start = true; //TODO Remove when limit switch added
            
            //armSpeed = home(); //TODO Uncomment when limit switch added
        } else {
            //Do not allow arm to start positioning unless homed first
            if(a_start){
                if(driver2.getRawButton(4)) {
                    //Lower Arm to intake posistion
                    setpoint = -500;
                    //Start intake
                    armBeltSpeed = ArmIntakeSpeed;
                } else {
                    //Chaser
                    if(driver2.getPOV() == 0) { // UP
                        setpoint = chaser(Direction.UP, rateOfChange);
                    } if(driver2.getPOV() == 180) { // DOWN 
                        setpoint = chaser(Direction.DOWN, rateOfChange);
                    } else { //HOLD & Uses default location at first
                        setpoint = chaser(Direction.NONE, 0.0);
                    }
                }

                armSpeed = -pid.calculate(armEncoder.get(), setpoint);
            }
        }

        double upperLimit = 0;
        double lowerLimit = -550; // TODO Find new Lower limit and update

        //Keep arm from over driving with soft stops
        if((armEncoder.get() >= lowerLimit && armEncoder.get() <= upperLimit) /* TODO Uncomment when limit switch added : || LimitSwitchTop.get() */) {
            /* 
            Not Done Yet!

            double armSpeedTemp = -pid.calculate(armEncoder.get(), chaser(Direction.NONE, 0.0));

            if (armEncoder.get() >= lowerLimit) {
                if(!(armEncoder.get() >= setpoint)) {
                    armSpeed = armSpeedTemp;
                }
            } */
        }
        
        Arm.set(armSpeed);
        Armbelt.set(armBeltSpeed);

        dashboard();
    }
}

