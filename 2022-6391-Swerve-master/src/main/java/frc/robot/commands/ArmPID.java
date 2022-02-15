/*
package frc.robot.commands;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;
import com.revrobotics.CANSparkMax;

public class ArmPID extends CommandBase {

    private final ArmControl armcontrol;
    private final double ArmSpeed, BeltSpeed;
    private SparkMaxPIDController Armpid;
    private boolean mode;
    
    public ArmPID(ArmControl armcontrol, double ArmSpeed, double BeltSpeed, boolean mode){
        this.armcontrol = armcontrol;
        this.ArmSpeed = ArmSpeed;
        this.BeltSpeed = BeltSpeed;
        this.mode = mode;
        addRequirements(armcontrol);
    }

    @Override 
    public void initialize(){
        Armpid = armcontrol.getPID();
        Armpid.setP(0.1);Armpid.setI(1e-4);Armpid.setD(1);Armpid.setIZone(0);Armpid.setFF(0);Armpid.setOutputRange(-1, 1);
    }


    @Override
    public void execute(){
        if( mode ){
            Armpid.setReference(2, CANSparkMax.ControlType.kPosition);
            armcontrol.MoveArmBelt(BeltSpeed);
        } else {
            armcontrol.MoveArm(ArmSpeed);
            armcontrol.MoveArmBelt(BeltSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
       armcontrol.MoveArm(0);
       armcontrol.MoveArmBelt(0);
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
}
*/