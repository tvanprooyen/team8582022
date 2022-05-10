
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

public class Conveyer extends SubsystemBase {
    //Proximity Sensors
    private DigitalInput sensor0 = new DigitalInput(ConveyorProximitySensor0);
    private DigitalInput sensor1 = new DigitalInput(ConveyorProximitySensor1);
    private DigitalInput sensor2 = new DigitalInput(ConveyorProximitySensor2);
    private DigitalInput sensor3 = new DigitalInput(ConveyorProximitySensor3);

    private final AnalogInput ultrasonic = new AnalogInput(2);

    //Motors
    private final CANSparkMax conveyer = new CANSparkMax(ConveyorID,MotorType.kBrushless);
    private final CANSparkMax shooter = new CANSparkMax(ShooterID,MotorType.kBrushless);

    private final CANSparkMax shooterBack = new CANSparkMax(40, MotorType.kBrushless);

    private double c_posistion;

    //Timers
    private final Joystick driver2 = new Joystick(Driver1Port);
    private final Timer timer = new Timer();
    private final Timer as1 = new Timer();

    private int ballCounter = -1;
    private boolean belt_run = false;

    //Speed Enum
    public enum Speed {
        REV(-1), STOP(0), LOW(1), HIGH(2);
    
        private final int value;
        private Speed(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    //Sensor Enum
    public enum ProxSensor {
        INIT(0), STATION1(1), STATION2(2), STATION3(3);
    
        private final int value;
        private ProxSensor(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    private boolean c_startup;

    //Shooter PID System
    private SparkMaxPIDController m_pidController;
    private SparkMaxPIDController m_pidController_Back;
    private final PIDController pid_convyor = new PIDController(0.1, 0, 0);
    private RelativeEncoder m_encoder;
    private RelativeEncoder m_encoder_back;
    private RelativeEncoder m_encoder_conveyor;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private boolean enableTeleop;

    public Conveyer(){
        SmartDashboard.putNumber("shooter",7);

        shooter.restoreFactoryDefaults();

        SmartDashboard.putNumber("shooter", 0);
    SmartDashboard.putNumber("back shooter", 0); 

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = shooter.getPIDController();

        m_pidController_Back = shooterBack.getPIDController();

        // Encoder object created to display position values
        m_encoder = shooter.getEncoder();

        m_encoder_back = shooterBack.getEncoder();

        m_encoder_conveyor = conveyer.getEncoder();


        // PID coefficients
        kP = 2e-5;
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        //Set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);


        // PID coefficients
        kP = 2e-5;
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        //Set PID coefficients
        m_pidController_Back.setP(kP);
        m_pidController_Back.setI(kI);
        m_pidController_Back.setD(kD);
        m_pidController_Back.setIZone(kIz);
        m_pidController_Back.setFF(kFF);
        m_pidController_Back.setOutputRange(kMinOutput, kMaxOutput);

        this.enableTeleop = true;

        m_encoder_conveyor.setPosition(0);

        c_posistion = m_encoder_conveyor.getPosition();

        this.c_startup = true;
    }

    public void enableTeleop(boolean enable) {
        this.enableTeleop = enable;
    }

    public boolean getEnableTeleop() {
        return this.enableTeleop;
    }

    public boolean getProxSensor0() {
        return sensor0.get();
    }

    public boolean getProxSensor1() {
        return sensor1.get();
    }

    public boolean getProxSensor2() {
        return sensor2.get();
    }

    public boolean getProxSensor3() {
        return sensor3.get();
    }

    public boolean getProxSensor(int sensor) {
        boolean value;

        switch (sensor) {
            case 1:
                value = getProxSensor1();
                break;

            case 2:
                value = getProxSensor2();
                break;

            case 3:
                value = getProxSensor3();
                break;
        
            default:
                value = getProxSensor0();
                break;
        }

        return value;
    }

    public boolean getProxSensor(ProxSensor sensor) {
        boolean value;

        switch (sensor) {
            case STATION1:
                value = getProxSensor(1);
                break;

            case STATION2:
                value = getProxSensor(2);
                break;

            case STATION3:
                value = getProxSensor(3);
                break;
        
            default:
                value = getProxSensor(0);
                break;
        }

        return value;
    }

    public void setShooterSpeed(double speed) {
        
        //Scaled as -1(-100%) thru 1(100%) (aka -5700RPM thru 5700RPM). Example) 0.25(25%) = 1425PRM
        double setPoint = speed*maxRPM;
        m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void setBackShooterSpeed(double speed) {
        
        //Scaled as -1(-100%) thru 1(100%) (aka -5700RPM thru 5700RPM). Example) 0.25(25%) = 1425PRM
        double setPoint = speed*11000; //3.3 == Max 
        m_pidController_Back.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void setConveyerSpeed(double speed){
        conveyer.set(speed);
    }

    public void setConveyerSpeed(Speed speedSelect) {
        switch (speedSelect) {
            case REV:
                setConveyerSpeed(Constants.ConveyorReverseSpeed);
                break;
            case LOW:
                setConveyerSpeed(Constants.ConveyorLowSpeed);
                break;
            case HIGH:
                setConveyerSpeed(Constants.ConveyorHighSpeed);
                break;
            default:
                setConveyerSpeed(0);
                break;
        }
    }

    public double getUSDistance(boolean METRIC) {
        double rawValue = ultrasonic.getValue();
        double value;

        //voltage_scale_factor allows us to compensate for differences in supply voltage.

        double voltage_scale_factor = 5/RobotController.getVoltage5V();

        if(METRIC) {
            value = rawValue * voltage_scale_factor * 0.125;
        } else {
            value = rawValue * voltage_scale_factor * 0.0492;
        }

        return value;
    }

    public double GetDistance(){
        double distance;

        double a2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        distance = (101.75-44.125)/(Math.tan((Math.PI/180)*(20+a2)));

       return distance;
    }

    public void dashboard() {
        //Send Vaules to Dashboard
        //Sensors
        //Prox Sensors
        //SmartDashboard.putBoolean("Prox Sensor | Initialize", getProxSensor(ProxSensor.INIT));
        //SmartDashboard.putBoolean("Prox Sensor | Station 1", getProxSensor(ProxSensor.STATION1));
        //SmartDashboard.putBoolean("Prox Sensor | Station 2", getProxSensor(ProxSensor.STATION2));
        //SmartDashboard.putBoolean("Prox Sensor | Station 3", getProxSensor(ProxSensor.STATION3));
        //Encoder
        SmartDashboard.putNumber("Distance", GetDistance());
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
        SmartDashboard.putNumber("ProcessVariableBack", m_encoder_back.getVelocity());
        SmartDashboard.putNumber("US Distance 1", getUSDistance(false));

        SmartDashboard.putNumber("ProcessVariableConvyor", m_encoder_conveyor.getPosition());



        //Timers
        //SmartDashboard.putNumber("Timer", timer.get());
        //For Automated Sequence # 1
        //SmartDashboard.putNumber("Shooter Timer", as1.get());
    }

    @Override
    public void periodic(){
        //Default Motors to Zero
        Speed c_speed = Speed.STOP;
        double s_speed = 0;
        double sb_speed = 0;
        double c_maxspeed = 0.25;

        if(driver2.getRawButton(6)){
            /*  Automated Sequence # 1
            *   1) Start Shooter, Stop Conveyor, Start Timer
            *   2) After 2.50 Start Conveyor
            *   3) After 2.65 Stop Conveyor
            *   4) After 4.00 Start Conveyor
            *   5) After 4.50 Stop Conveyor
            *
            *   Note(s):
            *   Starts from bottom and ends twords top
            */

           
            /*  old linear line
            
            double minspeed = 3.6;
            double maxspeed = 5.04 + 0.5;
            double mindist = 100;
            double maxdist = 180;
            double slope = (maxspeed-minspeed)/(maxdist-mindist);
             s_speed = (slope*GetDistance()) - (slope*mindist) + minspeed;
         */
         /*
            old polynomial regression
            s_speed = (0.000653222 * Math.pow(GetDistance(), 2)) - (0.121766 * GetDistance()) + 9.39559;
         */

           //s_speed = SmartDashboard.getNumber("shooter", 0);
           //sb_speed = SmartDashboard.getNumber("back shooter", 0); 

       //  s_speed = 0.00442393*GetDistance() + 1.62135;
       //  sb_speed  = 0.00180804*GetDistance() + 0.293508;

       s_speed = -0.00556715 * GetDistance() + 4.62829;
       sb_speed = 0.0153097 * GetDistance() + -1.54029;

       //s_speed = 1;
       //sb_speed = 0.5;

           if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 0 && getUSDistance(false) < 14) {
            s_speed = 1.5;
            sb_speed = 0.2;
            }

           if(driver2.getPOV() == 270) {
               s_speed = 3.75;
               sb_speed = 0.6;
           }
            
           c_maxspeed = 0.75;

            if(as1.get() == 0) {
                //Sequence #1
                as1.start();
                m_encoder_conveyor.setPosition(0);
                c_posistion = 0;
            }else if(as1.get() < 1.3) {

            } else if(as1.get() < 1.8) {
                //Sequence #3
                c_posistion = -7;
            } else if(as1.get() < 2) {
                //Sequence #3
                as1.stop();
                as1.reset();
            }

        } else if(driver2.getRawButton(2)) {
            //Remove Cargo 
            c_speed = Speed.REV;
        } else {
            as1.stop();
            as1.reset();
            
            /* if(sensor2.get()) {
                if(timer.get() == 0) {
                    timer.start();
                }
            }

            if(timer.get() >= 0.005) {
                c_speed = Speed.STOP;
                timer.stop();
            } else {
                if(!c_startup) {
                    c_speed = Speed.LOW;
                }
            }

            if(!getProxSensor(ProxSensor.STATION2)) {
                if(getProxSensor(ProxSensor.INIT)) {
                    timer.reset();
                    c_startup = false;
                }
            }
            

            if(getProxSensor(ProxSensor.INIT) && 
            getProxSensor(ProxSensor.STATION1) && 
            getProxSensor(ProxSensor.STATION2) && 
            getProxSensor(ProxSensor.STATION3)) {
                timer.reset();
            } */

            if(driver2.getRawButton(7)) {
                m_encoder_conveyor.setPosition(0);
                c_posistion = 0;
            }

            if(getProxSensor(ProxSensor.INIT)) {

                c_posistion = -10;
            } else if(getProxSensor(ProxSensor.STATION2)) {
                //c_posistion += 50; //c_startup = true;
                m_encoder_conveyor.setPosition(0);
                c_posistion = 0;
                ballCounter += 1;
            }

            if(getProxSensor(ProxSensor.STATION1)) {
                if(this.c_startup == true) {
                    ballCounter += 1;
                    this.c_startup = false;
                }
            }

            System.out.println(ballCounter);

            //m_pidController_conveyor.setReference(c_posistion, CANSparkMax.ControlType.kPosition);

            //ballCounter
        }
        //m_pidController_conveyor.setReference(c_posistion, CANSparkMax.ControlType.kPosition);

        double conveyor_speed = pid_convyor.calculate(m_encoder_conveyor.getPosition(), c_posistion);

        if(conveyor_speed < -c_maxspeed && conveyor_speed < 0) {
            conveyor_speed = -c_maxspeed;
        }

        if(conveyor_speed > c_maxspeed && conveyor_speed > 0) {
            conveyor_speed = c_maxspeed;
        }

        if(driver2.getRawButton(2)) {
            conveyor_speed = 0.25;
            m_encoder_conveyor.setPosition(0);
                c_posistion = 0;
        }

        conveyer.set(conveyor_speed);

        /* if(c_posistion >= -4) {
            m_encoder_conveyor.setPosition(0);
            c_posistion = 0;
        } */

        //setConveyerSpeed(c_speed);
        //Set Motor Speeds
        if(getEnableTeleop()) {
            setShooterSpeed(s_speed);
            setBackShooterSpeed(sb_speed);
        }
        

        //Send data to Dashboard
        dashboard();
    }
}
