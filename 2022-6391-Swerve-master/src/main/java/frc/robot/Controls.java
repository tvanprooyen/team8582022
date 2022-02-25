package frc.robot;

import com.team858.control.Joystick_858;

public final class Controls {
    
    //JoySticks
    public static final Joystick_858 driver1;
    //public static final Joystick_858 driver2 = new Joystick_858(1);

    static {
        driver1 = new Joystick_858(0);
    }
}
