package com.team858.control;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class Joystick_858 extends Joystick {
    //Slew Rate Limiter
    private double SRL_pOutput;

    //Axis Conditions
    private double deadband;
    private boolean modify;

    public Joystick_858(int port) {
        super(port);
        //Resort to Defaults
        this.deadband = Constants.deadband;
        this.modify = Constants.modify;
    }

    public Joystick_858(int port, double deadband) {
        super(port);
        this.deadband = deadband;
        this.modify = false;
    }

    public Joystick_858(int port, double deadband, boolean modify) {
        super(port);
        this.deadband = deadband;
        this.modify = modify;
    }


    @Override
    public double getRawAxis(int axis) {

        if(modify) {
            return ControlMathUtil.modifyAxis(getRawAxis(axis), deadband);
        }

        if(deadband != 0) {
            return ControlMathUtil.deadband(getRawAxis(axis), deadband);
        }

        return super.getRawAxis(axis);
    }
    
    //Slew Rate Limiter
    public double limiter(int axis, double maxChange) {
        double output = getRawAxis(axis);

            //Ramp Motor
            if (output > this.SRL_pOutput+maxChange) {
                output = this.SRL_pOutput+maxChange;
            } else if (output < this.SRL_pOutput-maxChange) {
                output = this.SRL_pOutput-maxChange;
            }

            //Save output for next iteration
            this.SRL_pOutput = output;
        
        return output;
    }

    public double limiter(int axis) {
        double output = getRawAxis(axis);
        double maxChange = Constants.changeRate;

            //Ramp Motor
            if (output > this.SRL_pOutput+maxChange) {
                output = this.SRL_pOutput+maxChange;
            } else if (output < this.SRL_pOutput-maxChange) {
                output = this.SRL_pOutput-maxChange;
            }

            //Save output for next iteration
            this.SRL_pOutput = output;
        
        return output;
    }
}