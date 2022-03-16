package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
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
        m_leftForwardLimit = leftLiftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        m_rightForwardLimit = rightLiftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

        //leftLiftMotor.getEncoder().setPosition(0);
        //leftLiftMotor.getEncoder().setPosition(0);

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

    public boolean getLeftLimitSwitch() {
        return m_leftForwardLimit.isPressed();
    }

    private double chaser(Direction direction, double rateOfChange, double chaser) {
        if(direction == Direction.UP) {
            chaser += rateOfChange;
        } else if (direction == Direction.DOWN) {
            chaser -= rateOfChange;
        }

        return chaser;
    }

    public void dashboard() {
        SmartDashboard.putBoolean("Right Lift Limit", getRightLimitSwitch());
        SmartDashboard.putBoolean("Right Soft Limit", getRightEncoder() <= -490);
        SmartDashboard.putNumber("Right Lift Encoder", rightLiftMotor.getEncoder().getPosition());

        SmartDashboard.putBoolean("Left Lift Limit", getLeftLimitSwitch());
        SmartDashboard.putBoolean("Left Soft Limit", getLeftEncoder() <= -490);
        SmartDashboard.putNumber("Left Lift Encoder", leftLiftMotor.getEncoder().getPosition());
    }

    @Override
    public void periodic(){
        boolean rightLimit = getRightLimitSwitch();
        double rightEncoder = getRightEncoder();
        boolean leftLimit = getLeftLimitSwitch();
        double leftEncoder = getLeftEncoder();
        double rateOfChange = 1;

        double axisRight = chaser(Direction.UP, driver1.getRawAxis(2) * rateOfChange, this.C_setpointRight) + chaser(Direction.DOWN, driver1.getRawAxis(3) * rateOfChange, this.C_setpointRight);
        double axisLeft = chaser(Direction.UP, driver1.getRawAxis(2) * rateOfChange, this.C_setpointLeft) + chaser(Direction.DOWN, driver1.getRawAxis(3) * rateOfChange, this.C_setpointLeft);
        
        double leftMotorSpeed = axisLeft;
        double rightMotorSpeed = axisRight;


        /* if(driver1.getRawAxis(2) > 0.2) {
            leftMotorSpeed = 0.5;
            rightMotorSpeed = 0.5; 
        } else if(driver1.getRawAxis(3) > 0.2) {
            leftMotorSpeed = -0.5;
            rightMotorSpeed = -0.5; 
        } */
        
        //-490
        boolean rightHighLimit = rightEncoder <= -290;
        if(rightLimit){
            rightLiftMotor.getEncoder().setPosition(0);
        }

        if(driver1.getRawButton(1)) {
            if(rightLimit && rightMotorSpeed > 0) {
                rightMotorSpeed = this.C_setpointRight;
            } 
            if(rightHighLimit && rightMotorSpeed < 0) {
                rightMotorSpeed = this.C_setpointRight;
            }
        } else {
            rightMotorSpeed = this.C_setpointRight;
        }
        this.C_setpointRight = rightMotorSpeed;
        rightLiftMotor.set(-pidRight.calculate(getRightEncoder(), rightMotorSpeed));

        boolean leftHighLimit = leftEncoder <= -290;
        if(leftLimit){
            leftLiftMotor.getEncoder().setPosition(0);

        }

        if(driver1.getRawButton(1)) {
            if(leftLimit && leftMotorSpeed > 0) {
                leftMotorSpeed = this.C_setpointLeft;
            } 
            if(leftHighLimit && leftMotorSpeed < 0) {
                leftMotorSpeed = this.C_setpointLeft;
            }
        } else {
                leftMotorSpeed = this.C_setpointLeft;
        }
        this.C_setpointLeft = leftMotorSpeed;
            //rightEncoder >= -490 ||
        leftLiftMotor.set(-pidLeft.calculate(getLeftEncoder(), leftMotorSpeed)); //May need to be positive
        dashboard();
    }
}