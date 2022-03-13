/*
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import static frc.robot.Constants.*;

public class ClimbingSubsys extends SubsystemBase {

   
    private final CANSparkMax leader = new CANSparkMax(leaderID,MotorType.kBrushless);
    private final CANSparkMax follower = new CANSparkMax(followerID,MotorType.kBrushless);

    private final RelativeEncoder leaderEncoder = leader.getEncoder();
    private final RelativeEncoder followerEncoder = follower.getEncoder();

    private final Joystick driver = new Joystick(Driver1Port);

    private final PIDController pidClimb = new PIDController(0.004, 0, 0);
    private double startLocation, fullLocation;

    public ClimbingSubsys(){

        follower.follow(leader);
        this.startLocation = 0;
        this.fullLocation = 1000;

    }

    @Override
    public void periodic(){
        double leaderSpeed = 0;
        double setPoint = this.startLocation;

        //reset encoders
        if(driver.getRawButton(6)){
            leaderEncoder.setPosition(0.0);
            followerEncoder.setPosition(0.0);

        //extend fully
        } else if(driver.getRawButton(8)){
            setPoint = this.fullLocation; 
        }

        leaderSpeed = pidClimb.calculate(leaderEncoder.getPosition(), setPoint);

        leader.set(leaderSpeed);
    }
}
*/