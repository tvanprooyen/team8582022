/* 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyer;
import edu.wpi.first.wpilibj.DigitalInput;

public class ConveyerCmd extends CommandBase {
    private final Conveyer convey;
    private final double speed;
    

    public ConveyerCmd(Conveyer convey, double speed) {
        this.convey = convey;
        this.speed = speed;
        addRequirements(convey);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        double c_speed = 0;

       if(!convey.getSensor3()){
        c_speed = -0.2;
       }

       convey.setSpeed(c_speed);
    }

    @Override
    public void end(boolean interrupted) {
        convey.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

 */