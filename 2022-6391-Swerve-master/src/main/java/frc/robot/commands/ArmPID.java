/*
package frc.robot.commands;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;
import com.revrobotics.CANSparkMax;

public class ArmPID extends CommandBase {

    private final ArmControl armcontrol;
    private final double ArmRotation;
    private SparkMaxPIDController Armpid;

    
    public ArmPID(ArmControl armcontrol, double ArmRotation){
        this.armcontrol = armcontrol;
        this.ArmRotation = ArmRotation;
        this.Armpid = armcontrol.getMotor().getPIDController();
        addRequirements(armcontrol);
    }

    @Override 
    public void initialize(){
        Armpid.setP(0.1);
        Armpid.setI(1e-4);
        Armpid.setD(1);
        Armpid.setIZone(0);
        Armpid.setFF(0);
        Armpid.setOutputRange(-1, 1);
    }


    @Override
    public void execute(){
        Armpid.setReference(ArmRotation, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void end(boolean interrupted) {
        Armpid.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
}
*/
