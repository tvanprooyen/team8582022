package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class ClimbingSubsys extends SubsystemBase {
    private final CANSparkMax leftLiftMotor = new CANSparkMax(Constants.LeftLiftID, MotorType.kBrushless);
    private final CANSparkMax rightLiftMotor = new CANSparkMax(Constants.RightLiftID, MotorType.kBrushless);
    private final Joystick driver1 = new Joystick(Constants.Driver1Port);
    private SparkMaxLimitSwitch m_leftForwardLimit;
    private SparkMaxLimitSwitch m_rightForwardLimit;
    private final PIDController pidRight = new PIDController(0.004, 0, 0);
    private final PIDController pidLeft = new PIDController(0.004, 0, 0);
    private double C_setpointLeft;
    private double C_setpointRight;

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

    public ClimbingSubsys() {
        m_leftForwardLimit = leftLiftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_rightForwardLimit = rightLiftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        C_setpointLeft = leftLiftMotor.getEncoder().getPosition();
        C_setpointRight = rightLiftMotor.getEncoder().getPosition();
    }

    public double getRightEncoder() {
        return leftLiftMotor.getEncoder().getPosition();
    }

    public double getLeftEncoder() {
        return rightLiftMotor.getEncoder().getPosition();
    }

    public boolean getRightLimitSwitch() {
        return m_rightForwardLimit.isPressed();
    }

    public boolean getRightSoftLimit() {
        return getRightEncoder() <= -450;
    }

    public boolean getLeftSoftLimit() {
        return getLeftEncoder() <= -450;
    }

    public boolean getLeftLimitSwitch() {
        return m_leftForwardLimit.isPressed();
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

    /**
     *Drives Climbers
     * 
     * @param setpoint Where the motor sould drive to
     * @param button Saftey button
     * @param direction What side of robot to control
     */
    private void runClimbSystem(double setpoint, Boolean button, Direction direction) {

        double masterSetpoint, encoderPos;
        boolean topLimit, bottomLimit;
        CANSparkMax liftMotor;
        PIDController pidController;
        RelativeEncoder encoder;

        //Settings
        switch (direction) {
            case RIGHT:
                liftMotor = rightLiftMotor;
                masterSetpoint = this.C_setpointRight;
                encoder = liftMotor.getEncoder();
                topLimit = getRightSoftLimit();
                bottomLimit = getRightLimitSwitch();
                pidController = pidRight;
                encoderPos = getRightEncoder();
                break;
            case LEFT:
                liftMotor = leftLiftMotor;
                masterSetpoint = this.C_setpointLeft;
                encoder = leftLiftMotor.getEncoder();
                topLimit = getLeftSoftLimit();
                bottomLimit = getLeftLimitSwitch();
                pidController = pidLeft;
                encoderPos = getLeftEncoder();
                break;
            default:
                return;
        }
        
        //Set to Zero
        if(bottomLimit){
            encoder.setPosition(0);
        }

        //Stop at Limits
        if(button) {
            if(topLimit && setpoint > 0) {
                setpoint = masterSetpoint;
            } 
            if(bottomLimit && setpoint < 0) {
                setpoint = masterSetpoint;
            }
        } else {
            setpoint = masterSetpoint;
        }
        
        //Drive Motor
        liftMotor.set(-pidController.calculate(encoderPos, setpoint)); //TODO May need to be positive
    }

    public void dashboard() {
        SmartDashboard.putBoolean("Right Lift Limit", getRightLimitSwitch());
        SmartDashboard.putBoolean("Right Soft Limit", getRightSoftLimit());
        SmartDashboard.putNumber("Right Lift Encoder", rightLiftMotor.getEncoder().getPosition());

        SmartDashboard.putBoolean("Left Lift Limit", getLeftLimitSwitch());
        SmartDashboard.putBoolean("Left Soft Limit", getLeftSoftLimit());
        SmartDashboard.putNumber("Left Lift Encoder", leftLiftMotor.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        double rateOfChange = 1;

        double axisRight = chaser(Direction.UP, driver1.getRawAxis(2) * rateOfChange, this.C_setpointRight) + chaser(Direction.DOWN, driver1.getRawAxis(3) * rateOfChange, this.C_setpointRight);
        double axisLeft = chaser(Direction.UP, driver1.getRawAxis(2) * rateOfChange, this.C_setpointLeft) + chaser(Direction.DOWN, driver1.getRawAxis(3) * rateOfChange, this.C_setpointLeft);

        runClimbSystem(axisLeft, driver1.getRawButton(1), Direction.LEFT);
        runClimbSystem(axisRight, driver1.getRawButton(1), Direction.RIGHT);

        dashboard();

        //Old Code to fall back on too
        /* //Right
        if(getRightSoftLimit()){
            rightLiftMotor.getEncoder().setPosition(0);
        }

        if(driver1.getRawButton(1)) {
            if(getRightSoftLimit() && rightMotorSpeed > 0) {
                rightMotorSpeed = 0;
            } 
            if(getRightSoftLimit() && rightMotorSpeed < 0) {
                rightMotorSpeed = 0;
            }
        } else {
            rightMotorSpeed = 0;
        }

        rightLiftMotor.set(rightMotorSpeed);

        //Left
        if(getLeftLimitSwitch()){
            leftLiftMotor.getEncoder().setPosition(0);

        }

        if(driver1.getRawButton(1)) {
            if(getLeftLimitSwitch() && leftMotorSpeed > 0) {
                leftMotorSpeed = 0;
            } 
            if(getLeftSoftLimit() && leftMotorSpeed < 0) {
                leftMotorSpeed = 0;
            }
        } else {
                leftMotorSpeed = 0;
        }
        
        leftLiftMotor.set(leftMotorSpeed); */
    }
}