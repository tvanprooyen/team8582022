package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase {

    //Speed Enum
    public enum AutoMode {
        NONE(0), AutoMode1(1), AutoMode2(2), AutoMode3(3);
    
        private final int value;
        private AutoMode(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    //public void atuo


    Auto() {

    }

    public void dashboard() {

    }

    @Override
    public void periodic() {
        //AutoMode autoMode = AutoMode.valueOf(SmartDashboard.getNumber("Auto Mode", 0));
        
        dashboard();
    }
    
}
