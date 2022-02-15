/*
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyer;

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
        convey.setSpeed(speed);
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