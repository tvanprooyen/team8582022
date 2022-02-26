package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

public class Conveyer extends SubsystemBase {  
    private DigitalInput sensor3 = new DigitalInput(3);
    private DigitalInput sensor0 = new DigitalInput(0);
    private DigitalInput sensor1 = new DigitalInput(1);
    private DigitalInput sensor2 = new DigitalInput(2);
    private final CANSparkMax conveyer = new CANSparkMax(ConveyorID,MotorType.kBrushless);
    private final CANSparkMax Shooter = new CANSparkMax(ShooterID,MotorType.kBrushless);
    private final Joystick driver2 = new Joystick(1);
    private final Timer timer = new Timer();
    private final Timer shootTimer = new Timer();

    private boolean c_startup = true;

    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    
    private boolean latch;

    public Conveyer(){
        SmartDashboard.putNumber("shooter",7);

        Shooter.restoreFactoryDefaults();

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = Shooter.getPIDController();

        // Encoder object created to display position values
        m_encoder = Shooter.getEncoder();

        // PID coefficients
        kP = 2e-5;
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
    }

    /* public boolean getSensor0(){
        return sensor0.get();
    }

    public boolean getSensor1(){
        return sensor1.get();
    }

    public boolean getSensor2(){
        return sensor2.get();
    } */

    public boolean getSensor3(){
        return sensor3.get();
    }

    /* public boolean getSensor4(){
        return sensor4.get();
    }
 */
    public boolean latch(boolean latch) {
        this.latch = false;
        return latch;
    }

    public boolean getLatch() {
        return this.latch;
    }
/*
    public void gate() {
        if( 
            (!getSensor0() && !getSensor1()) ||
            (getLatch() && getSensor1())

        ) latch(false);  return;
        
        if(
            getSensor0() ||
            (getSensor0() && getSensor1())
        ) latch(true); return;
    }
*/
    @Override
    public void periodic(){
        double c_speed = 0;
        double s_speed = 0;
        boolean beforemotor = sensor3.get();

        SmartDashboard.putBoolean("Sensor3", beforemotor);
        SmartDashboard.putNumber("Timer", timer.get());

        if(driver2.getRawButton(1)){
            s_speed = SmartDashboard.getNumber("shooter",0);

            if(shootTimer.get() == 0) {
                shootTimer.start();
                c_speed = 0;
            } else if(shootTimer.get() >= 4.5) {
                c_speed = 0;
            }else if(shootTimer.get() >= 4) {
                c_speed = -0.7;
            } else if(shootTimer.get() >= 2.65) {
                c_speed = 0;
            } else if(shootTimer.get() >= 2.5) {
                c_speed = -0.7;
            }
        } else if(driver2.getRawButton(2)) {
            c_speed = 0.2;
        } else {
            shootTimer.stop();
            shootTimer.reset();
            
            if(sensor2.get()) {
                if(timer.get() == 0) {
                    timer.start();
                }
            }

            if(timer.get() >= 0.005) {
                c_speed = 0;
                timer.stop();
            } else {
                if(!c_startup) {
                    c_speed = -0.25;
                }
            }

            if(!sensor3.get()) {
                if(sensor0.get()) {
                    timer.reset();
                    c_startup = false;
                }
            }
            

            if(sensor0.get() && sensor1.get() && sensor2.get() && sensor3.get()) {
                timer.reset();
            }
        }

        
        


        /* if(driver2.getRawButton(2) || timer.get() != 0) {
            //if(timer.get() <= 500 && timer.get() != 0){
            c_speed = -0.2;
            //}
        } */
        
        /* if(driver2.getRawButton(3)) {
            timer.stop();
            timer.reset();
            c_speed = -0.3;
        } else {
            timer.stop();
            timer.reset();
        } */

        /* if(driver2.getRawButton(2)) {
            if(!beforemotor){
                if(timer.get() == 0) {
                    timer.start();
                }
            }
            if(timer.get() >= 500) {
                c_speed = 0;
                timer.stop();
                timer.reset();
            } else if(timer.get() <= 500 && timer.get() != 0){
                c_speed = -0.2;
            }
        } else  */

        
        //Shooter.set(s_speed);
        conveyer.set(c_speed);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    double setPoint = s_speed*maxRPM;
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }
    
    

    public void setSpeed(double speed){
         conveyer.set(speed);
    } 
}

