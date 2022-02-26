
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
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
    private final Encoder armEncoder = new Encoder(5, 6, 7, false);
    private final Joystick driver2 = new Joystick(1);
    private final PIDController pid = new PIDController(0.004, 0, 0);

    private boolean a_start = false;

    public ArmControl(){
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


    @Override
    public void periodic(){

        /* double Kp = -0.05f;
        double min_command = 0.05f;
         
        double a_heading = armEncoder.get();

        //PID
        double heading_error = -a_heading;
        double arm_adjust = 0.0f;
        if (a_heading > 1.0){
            arm_adjust = Kp*heading_error - min_command;
        }else if (a_heading < 1.0){
            arm_adjust = Kp*heading_error + min_command;
        } */

        if(driver2.getRawButton(7)) {
            armEncoder.reset();
            a_start = true;
        }

        if(driver2.getRawButton(4)) {
            Arm.set(-pid.calculate(armEncoder.get(), -500));
            Armbelt.set(0.3);
        } else {
            if(a_start){
                Arm.set(-pid.calculate(armEncoder.get(), -40));
            } else {
                Arm.set(0);
            }
            Armbelt.set(0);
        }

        SmartDashboard.putNumber("Arm Encoder", armEncoder.get());
    }
}

