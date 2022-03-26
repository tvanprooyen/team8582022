
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;

import static frc.robot.Constants.*;

public class ArmControl extends SubsystemBase {
    private final CANSparkMax Arm = new CANSparkMax(ArmID,MotorType.kBrushless);
    private final CANSparkMax Armbelt = new CANSparkMax(ArmBeltID,MotorType.kBrushless);
    
    //Magnetic Limit Switches
    private DigitalInput LimitSwitchTop = new DigitalInput(ArmMagneticLimitSwitchTop);
    //private DigitalInput LimitSwitchBottom = new DigitalInput(ArmMagneticLimitSwitchBottom);

    private final Encoder armEncoder = new Encoder(ArmEncoderA, ArmEncoderB, ArmEncoderI, false);
    private final Joystick driver2 = new Joystick(Driver1Port);
    private final PIDController pid = new PIDController(0.004, 0, 0);

    private final AnalogInput ultrasonic = new AnalogInput(0);

    private boolean a_start = false;

    private double a_chaser, a_pickupSetpoint;

    private boolean enableTeleop;

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
        this.a_chaser = -10;
        this.a_pickupSetpoint = -470; //-550
        this.enableTeleop = true;
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

    public void setDriveArmMotor(Boolean up) {
        double speed = 0;
        if(up) {
            speed = -pid.calculate(armEncoder.get(), this.a_chaser);
        } else {
            speed = -pid.calculate(armEncoder.get(), this.a_pickupSetpoint);
        }
        Arm.set(speed);
    }

    public void setDriveArmIntakeMotor(double speed) {
        Armbelt.set(speed);
    }

    public void enableTeleop(boolean enable) {
        this.enableTeleop = enable;
    }

    public boolean getEnableTeleop() {
        return this.enableTeleop;
    }

    /**
   * Allows user to manually change a PID setpoint
   *
   * @param direction Direction UP(plus), DOWN(minus), NONE(nothing happens)
   * @param rateOfChange How much is added to the old setpoint
   * @return The next setpoint
   */
    private double chaser(Direction direction, double rateOfChange, double chaser) {
        if(direction == Direction.UP) {
            chaser += rateOfChange;
        } else if (direction == Direction.DOWN) {
            chaser -= rateOfChange;
        }

        return chaser;
    }

    public double getUSDistance(boolean METRIC) {
        double rawValue = ultrasonic.getValue();
        double value;

        //voltage_scale_factor allows us to compensate for differences in supply voltage.

        double voltage_scale_factor = 5/RobotController.getVoltage5V();

        if(METRIC) {
            value = rawValue * voltage_scale_factor * 0.125;
        } else {
            value = rawValue * voltage_scale_factor * 0.0492;
        }

        return value;
    }

    public double home() {
        if(LimitSwitchTop.get()) {
            armEncoder.reset();
            a_start = true;
            return 0;
        } else {
            return 0.3;
        }
    }


    public void dashboard() {
        //Send Vaules to Dashboard
        SmartDashboard.putNumber("Arm Encoder", armEncoder.get());
        SmartDashboard.putBoolean("Arm Limit Switch", LimitSwitchTop.get());
        SmartDashboard.putNumber("Distance Measure", getUSDistance(false));
    }

    @Override
    public void periodic(){
        double armSpeed = 0;
        double armBeltSpeed = 0;
        double rateOfChange = 2;
        double setpoint = this.a_chaser;

        if(driver2.getPOV() == 90) {
            //a_start = true;
        }

        if(driver2.getRawButton(8)) {
            /* if(!LimitSwitchTop.get()) {
                armEncoder.reset();
                a_start = true;
                armSpeed = 0;
            } else {
                armSpeed = -0.2;
            } */
            armSpeed = -0.2;
        } else {
            //Do not allow arm to start positioning unless homed first
            if(a_start){
                if(driver2.getRawButton(5) /* && !(getUSDistance(false) <= 30 ) */) {
                    //Lower Arm to intake posistion
                    setpoint = this.a_pickupSetpoint;

                    //Chaser
                    if(driver2.getPOV() == 0) { // UP
                        setpoint = chaser(Direction.UP, rateOfChange, this.a_pickupSetpoint);
                    } if(driver2.getPOV() == 180) { // DOWN 
                        setpoint = chaser(Direction.DOWN, rateOfChange, this.a_pickupSetpoint);
                    } else if (driver2.getPOV() == -1) { //HOLD & Uses default location at first
                        setpoint = chaser(Direction.NONE, 0.0, this.a_pickupSetpoint);
                    }

                    this.a_pickupSetpoint = setpoint;

                    //Start intake
                    armBeltSpeed = ArmIntakeSpeed;
                } else {
                    //Chaser
                    if(driver2.getPOV() == 0) { // UP
                        setpoint = chaser(Direction.UP, rateOfChange, this.a_chaser);
                    } if(driver2.getPOV() == 180) { // DOWN 
                        setpoint = chaser(Direction.DOWN, rateOfChange, this.a_chaser);
                    } else if (driver2.getPOV() == -1) { //HOLD & Uses default location at first
                        setpoint = chaser(Direction.NONE, 0.0, this.a_chaser);
                    }

                    this.a_chaser = setpoint;
                }

                armSpeed = -pid.calculate(armEncoder.get(), setpoint);
            }
        }

        if(LimitSwitchTop.get()) {
                armEncoder.reset();
                a_start = true;
        } else {
            if(!a_start){
                armSpeed = 0;
            }
        }

        double upperLimit = -10;
        double lowerLimit = -623;

        //Keep arm from over driving with soft stops
        if(a_start || (armEncoder.get() >= lowerLimit && armEncoder.get() <= upperLimit) /* TODO Uncomment when limit switch added : || LimitSwitchTop.get() */) {
            //SmartDashboard.putBoolean("Limit Hit", true);
            
            /* double armSpeedTemp = -pid.calculate(armEncoder.get(), chaser(Direction.NONE, 0.0));
            if (armEncoder.get() >= lowerLimit) {
                if(!(armEncoder.get() >= setpoint)) {
                    //armSpeed = armSpeedTemp;
                }
            } */
            
        } else {
            //SmartDashboard.putBoolean("Limit Hit", false);
        }

        /* if(armSpeed != 0) {
            if(Arm.getEncoder().getVelocity() >= 10) {
                armSpeed = 0;
                System.out.println("Motor Stalled");
            }
        } */
        
        if(getEnableTeleop()) {
            Arm.set(armSpeed);
            Armbelt.set(armBeltSpeed);
        }
        

        dashboard();
    }
}