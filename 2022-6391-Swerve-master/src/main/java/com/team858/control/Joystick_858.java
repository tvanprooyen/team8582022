package com.team858.control;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class Joystick_858 {
    //Slew Rate Limiter
    private double SRL_pOutput;

    //Axis Conditions
    private double deadband;
    private boolean modify;
    private int m_port;

    private Joystick driver = new Joystick(m_port);

    public Joystick_858(int port) {
        this.m_port = port;
        //Resort to Defaults
        this.deadband = Constants.deadband;
        this.modify = Constants.modify;
    }

    public Joystick_858(int port, double deadband) {
        this.deadband = deadband;
        this.modify = false;
    }

    public Joystick_858(int port, double deadband, boolean modify) {
        this.deadband = deadband;
        this.modify = modify;
    }


    public double getRawAxis(int axis) {

        double value = driver.getRawAxis(axis);

        if(modify) {
            return ControlMathUtil.modifyAxis(driver.getRawAxis(axis), deadband);
        }

        if(deadband != 0) {
            return ControlMathUtil.deadband(driver.getRawAxis(axis), deadband);
        }

        return value;
    }
    
    //Slew Rate Limiter
    public double limiter(int axis, double maxChange) {
        double output = driver.getRawAxis(axis);

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