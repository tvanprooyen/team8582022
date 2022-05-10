package com.team858.control;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    double Kp, min_command;
    double cameraHeight, goalHeight, cameraAngle;

    public LimeLight() {}

    public LimeLight(double Kp, double min_command) {
        this.Kp = Kp;
        this.min_command = min_command;
    }

    public LimeLight(double cameraHeight, double goalHeight, double cameraAngle) {
        this.cameraHeight = cameraHeight;
        this.goalHeight = goalHeight;
        this.cameraAngle = cameraAngle;
    }

    public LimeLight(double Kp, double min_command, double cameraHeight, double goalHeight, double cameraAngle) {
        this.Kp = Kp;
        this.min_command = min_command;
        this.cameraHeight = cameraHeight;
        this.goalHeight = goalHeight;
        this.cameraAngle = cameraAngle;
    }

    /**
   * DATA
   * <p><b>TV</b> - Whether the limelight has any valid targets (0 or 1)
   * <p><b>TX</b> - Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
   * <p><b>TX</b> - Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
   * <p><b>TY</b> - Target Area (0% of image to 100% of image)
   * <p><b>TA</b> - Skew or rotation (-90 degrees to 0 degrees)
   * <p><b>TS</b> - The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
   * <p><b>TL</b> - Sidelength of shortest side of the fitted bounding box (pixels)
   * <p><b>TSHORT</b> - Sidelength of longest side of the fitted bounding box (pixels)
   * <p><b>TLONG</b> - Horizontal sidelength of the rough bounding box (0 - 320 pixels)
   * <p><b>THOR</b> - Vertical sidelength of the rough bounding box (0 - 320 pixels)
   * <p><b>GETPIPE</b> - True active pipeline index of the camera (0 .. 9)
   * <p><b>CAMTRAN</b> - Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
   * <p><b>TC</b> - Get the average HSV color underneath the crosshair region as a NumberArray
   */
    public enum DATA {
        TV(0), TX(1), TY(2), TA(3), TS(4), TL(5), TSHORT(6), TLONG(7), THOR(8), TVERT(9), GETPIPE(10), CAMTRAN(11), TC(12);
    
        private final int value;

        private DATA(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }

        public String getName() {
            String name = "";
            switch (value) {
                case 0: name = "tv"; break;
                case 1: name = "tx"; break;
                case 2: name = "ty"; break;
                case 3: name = "ta"; break;
                case 4: name = "ts"; break;
                case 5: name = "tl"; break;
                case 6: name = "tshort"; break;
                case 7: name = "tlong"; break;
                case 8: name = "thor";  break;
                case 9: name = "tvert"; break;
                case 10: name = "gitpipe"; break;
                case 11: name = "camtran"; break;
                case 12: name = "tc"; break;
                default: name = ""; break;
            }
            return name;
        }
    }

    /**
   * CONTROLS
   * <p><b>LEDMODE</b> - Sets limelight’s LED state
   * <p><b>CAMMODE</b> - Sets limelight’s operation mode
   * <p><b>PIPELINE</b> - Sets limelight’s current pipeline
   * <p><b>STREAM</b> - Sets limelight’s streaming mode
   * <p><b>SNAPSHOT</b> - Allows users to take snapshots during a match
   */
    public enum CONTROLS {
        LEDMODE(0), CAMMODE(1), PIPELINE(2), STREAM(3), SNAPSHOT(4);
    
        private final int value;
        private CONTROLS(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }

        public String getName() {
            String name = "";
            switch (value) {
                case 0: name = "ledMode"; break;
                case 1: name = "camMode"; break;
                case 2: name = "pipeline"; break;
                case 3: name = "stream"; break;
                case 4: name = "snapshot"; break;
                default: name = ""; break;
            }
            return name;
        }
    }


    /**
     * Turns the leds on and off
     * 
     * @param mode true for on; false for Off
     */
    public void swtichLight(boolean mode) {
        int data = 1;
        if(mode) {
            data = 3;
        }

        setLimeLight(CONTROLS.LEDMODE, data);
    }

    /**
   * Gets Limelight Data
   *
   * @param data Select data
   * @return Limelight Data
   */
    public double getLimelight(DATA data) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(data.getName()).getDouble(0);
    }

    /**
   * Sets Limelight Data
   *
   * @param controls Select control
   * @param change Value to change
   */
    public void setLimeLight(CONTROLS controls, int change) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(controls.getName()).setNumber(change);
    }

    /**
   * Tracks Target
   *
   * @param steering Input steer
   * @return Output steer
   */
    public double trackTarget(double steering) {
        //Target
        double tx = getLimelight(DATA.TX);

        //PID
        double heading_error = -tx;
        double steering_adjust = 0.0f;
        if (tx > 1.0){
                steering_adjust = Kp*heading_error - min_command;
        } else if (tx < 1.0){
                steering_adjust = Kp*heading_error + min_command;
        }

        //Activly adjust twords target
        steering += steering_adjust;

        return steering;
    }

    /**
   * Tracks Target
   *
   * @param steering Input steer
   * @param Kp Proportional gain
   * @param min_command Adds the minimum power needed to move
   * @return Output steer
   */
    public double trackTarget(double steering, double Kp, double min_command) {
        //Target
        double tx = getLimelight(DATA.TX);

        //PID
        double heading_error = -tx;
        double steering_adjust = 0.0f;
        if (tx > 1.0){
                steering_adjust = Kp*heading_error - min_command;
        } else if (tx < 1.0){
                steering_adjust = Kp*heading_error + min_command;
        }

        //Activly adjust twords target
        steering += steering_adjust;

        return steering;
    }

    /**
   * Finds target distance
   *
   * @param cameraHeight Height of camera
   * @param goalHeight height of goal
   * @param cameraAngle Angle of camera
   * @return Distance
   */
    public double getDistance(double cameraHeight, double goalHeight, double cameraAngle){
        double distance;

        double targetAngle = getLimelight(DATA.TY);
        distance = (goalHeight-cameraHeight)/(Math.tan((Math.PI/180)*(cameraAngle+targetAngle)));

       return distance;
    }

    /**
   * Finds target distance
   *
   * @return Distance
   */
    public double getDistance(){
        double distance;

        double targetAngle = getLimelight(DATA.TY);
        distance = (goalHeight-cameraHeight)/(Math.tan((Math.PI/180)*(cameraAngle+targetAngle)));

       return distance;
    }
}
