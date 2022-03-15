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
import static frc.robot.Constants.*;

public class ClimbingSubsys extends SubsystemBase {
    private final CANSparkMax leftLiftMotor = new CANSparkMax(Constants.LeftLiftID, MotorType.kBrushless);
    private final CANSparkMax rightLiftMotor = new CANSparkMax(Constants.RightLiftID, MotorType.kBrushless);
    private final Joystick driver1 = new Joystick(Constants.Driver1Port);
    private SparkMaxLimitSwitch m_leftForwardLimit;
    private SparkMaxLimitSwitch m_rightForwardLimit;

    private final Joystick driver = new Joystick(Driver1Port);

    private final PIDController pidClimb = new PIDController(0.004, 0, 0);
    private double startLocation, fullLocation;

    public ClimbingSubsys() {
        m_leftForwardLimit = leftLiftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        m_rightForwardLimit = rightLiftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
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
        double leftMotorSpeed = driver1.getRawAxis(0);
        double rightMotorSpeed = driver1.getRawAxis(0);
        
        //-490
        boolean rightHighLimit = rightEncoder <= -490;
        if(rightLimit){
            rightLiftMotor.getEncoder().setPosition(0);
        }

        boolean leftHighLimit = leftEncoder <= -490;
        if(leftLimit){
            leftLiftMotor.getEncoder().setPosition(0);
        }

        if(driver1.getRawButton(1)) {
        if(rightLimit && rightMotorSpeed > 0) {
            rightMotorSpeed = 0;
        } 
        if(rightHighLimit && rightMotorSpeed < 0) {
            rightMotorSpeed = 0;
        }
        } else {
            rightMotorSpeed = 0;
        }
        rightLiftMotor.set(rightMotorSpeed);


        if(driver1.getRawButton(1)) {
            if(rightLimit && leftMotorSpeed > 0) {
                leftMotorSpeed = 0;
            } 
            if(leftHighLimit && leftMotorSpeed < 0) {
                leftMotorSpeed = 0;
            }
            } else {
                leftMotorSpeed = 0;
            }
            //rightEncoder >= -490 || 
            leftLiftMotor.set(leftMotorSpeed);
        dashboard();
    }
}