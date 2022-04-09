
package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import com.team858.control.LimeLight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDrive extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Conveyer m_conveyer;
    private final ArmControl m_arm;
    private PIDController gyroPID = new PIDController(1, 0, 0);  // 0.004 //1 //0.004
    private PIDController xPID = new PIDController(5, 0, 0);  // 0.004
    private PIDController yPID = new PIDController(0.01, 0, 0);  // 0.004 //
    private final Timer timer = new Timer();
    private final Timer as1 = new Timer();
    private final LimeLight limeLight = new LimeLight(-0.02f, 0.02f, //Target Steer PID
                                                        44.125, 101.75, 20); // Target Distance
    private int seq = 0;
    private int nextSeq = 0;
    private int saveSeq = 0;

    private boolean SeqToggle = false;

    private final SendableChooser<Integer> m_chooser;

    private float masterAngle;
    
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

    public AutoDrive(DrivetrainSubsystem drivetrainSubsystem,
                               Conveyer conveyer, 
                               ArmControl arm,
                               SendableChooser<Integer> chooser) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_conveyer = conveyer;
        this.m_arm = arm;
        this.m_chooser = chooser;
        addRequirements(drivetrainSubsystem);
        addRequirements(conveyer);
        addRequirements(arm);
    }

    private void drive(double x, double y, double rotation) {
        Rotation2d gyro = m_drivetrainSubsystem.getGyroscopeRotation();

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        rotation,
                        gyro
                )
        );
    }

    private void drive(double x, double y, double rotation, boolean robotOrient) {
        Rotation2d gyro = m_drivetrainSubsystem.getGyroscopeRotation();

        if(robotOrient) {
            gyro = Rotation2d.fromDegrees(0);
        }

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        rotation,
                        gyro
                )
        );
    }

    @Override
    public void initialize() {
        timer.start();
        m_conveyer.enableTeleop(false);
        m_arm.enableTeleop(false);
        this.masterAngle = m_drivetrainSubsystem.getGyro();
        m_drivetrainSubsystem.zeroGyroscope();
        m_drivetrainSubsystem.resetDriveEncoders();
        super.initialize();
    }

    private void auto1() {
        Conveyer.Speed c_speed = Conveyer.Speed.STOP;

        //linear line
        double minspeed = 3.6;
        double maxspeed = 5.04;
        double mindist = 100;
        double maxdist = 180;
        double slope = (maxspeed-minspeed)/(maxdist-mindist);
        double s_speed = 3.6;/* (slope*limeLight.getDistance()) - (slope*mindist) + minspeed */;
        float gyro = m_drivetrainSubsystem.getGyro();

        if(timer.get() < 0.75) {
            drive(-20, 0, -gyroPID.calculate(gyro, 0 /* this.masterAngle */) * DrivetrainSubsystem.MaxAngularVelocity, true);
            c_speed = Conveyer.Speed.STOP;
        } /* if(timer.get() < 2) {
            
        } */ else if(timer.get() < 4) {
            //drive(0, 0, (limeLight.trackTarget(0) * DrivetrainSubsystem.MaxAngularVelocity));
            c_speed = Conveyer.Speed.STOP;
        } else if(timer.get() < 4.5) {
            //Sequence #1
            drive(0, 0, 0);
            //s_speed = (slope*limeLight.getDistance()) - (slope*mindist) + minspeed;
            c_speed = Conveyer.Speed.STOP;
         } else if(timer.get() <= 5.8) {
            //Sequence #2
            c_speed = Conveyer.Speed.HIGH;
        } else if(timer.get() <= 5.90) {
            //Sequence #3
            c_speed = Conveyer.Speed.STOP;
        } else {
            drive(0, 0, 0);
            c_speed = Conveyer.Speed.STOP;
            s_speed = 0;
        }

        m_conveyer.setConveyerSpeed(c_speed);
        m_conveyer.setShooterSpeed(s_speed);
    }


    private void auto2() {
        Conveyer.Speed c_speed = Conveyer.Speed.STOP;

        //linear line
        /* double minspeed = 3.75;
        double maxspeed = 5.04;
        double mindist = 100;
        double maxdist = 180;
        double slope = (maxspeed-minspeed)/(maxdist-mindist);
        double s_speed = (slope*m_conveyer.GetDistance()) - (slope*mindist) + minspeed; */
        double s_speed = -0.00556715 * m_conveyer.GetDistance() + 4.62829;
        double sb_speed = 0.0153097 * m_conveyer.GetDistance() + -1.54029;

        float gyro = m_drivetrainSubsystem.getGyro();
        boolean up = true;
        double intake = 0;
        double x = 0;
        double y = 0;
        double rotation = 0;
        boolean robotOrt = false;

        if(timer.get() < 0.1) {
            up = false;
            intake = -0.3;
         } else if(timer.get() < 3) {
            //if(m_arm.getUSDistance(false) <= 33) {
             //  x = 0; y = 0; rotation = 0; robotOrt = false; // -gyroPID.calculate(gyro,this.masterAngle + 179.9) 
            //} else {
                x = 30; y = 0; rotation = -gyroPID.calculate(gyro, 0/* this.masterAngle */); robotOrt = true;      
            //}
            up = false;
            intake = -0.3;
            s_speed = 0;
            sb_speed = 0;
        } else if(timer.get() < 3.1) {
            up = true;
            intake = 0;
            s_speed = 0;
            sb_speed = 0;
            x = 0; y = 0; rotation = 0; robotOrt = false;       
        } else if(timer.get() < 5) {
            up = true;
            intake = 0;
            s_speed = 0;
            sb_speed = 0;
            //drive(0, 0, -gyroPID.calculate(gyro,this.masterAngle + 179.9), true); 
            x = 0; y = 0; rotation = -gyroPID.calculate(gyro,/* this.masterAngle -  */179.9); robotOrt = true;
        } else if(timer.get() < 7) {
            x = 0; y = 0; rotation = -(limeLight.trackTarget(0) * DrivetrainSubsystem.MaxVelocity); robotOrt = false;
            //drive(0, 0, (limeLight.trackTarget(0) * DrivetrainSubsystem.MaxAngularVelocity), false); 
        } else if(timer.get() < 9) {
            x = 0; y = 0; rotation = 0; robotOrt = false;
        } else if(timer.get() < 10) {
            x = 0; y = 0; rotation = 0; robotOrt = false;
            c_speed = Conveyer.Speed.HIGH;
            m_conveyer.setConveyerSpeed(c_speed);
        } else {
            c_speed = Conveyer.Speed.STOP;
            s_speed = 0;
            sb_speed = 0;
            x = 0; y = 0; rotation = 0; robotOrt = false;
        }
        
        m_conveyer.setBackShooterSpeed(sb_speed);
        m_conveyer.setShooterSpeed(s_speed);

        m_arm.setDriveArmMotor(up);
        m_arm.setDriveArmIntakeMotor(intake);

        drive(x, y, rotation, robotOrt); 
    }

    private void auto3() {
        Conveyer.Speed c_speed = Conveyer.Speed.STOP;

        //linear line
        /* double minspeed = 3.75;
        double maxspeed = 5.04;
        double mindist = 100;
        double maxdist = 180;
        double slope = (maxspeed-minspeed)/(maxdist-mindist);
        double s_speed = (slope*m_conveyer.GetDistance()) - (slope*mindist) + minspeed; */
        double s_speed = -0.00556715 * m_conveyer.GetDistance() + 4.62829;
        double sb_speed = 0.0153097 * m_conveyer.GetDistance() + -1.54029;
        float gyro = m_drivetrainSubsystem.getGyro();
        boolean up = true;
        double intake = 0;
        double x = 0;
        double y = 0;
        double rotation = 0;
        boolean robotOrt = false;

        if(timer.get() < 0.1) {
            up = false;
            intake = -0.3;
         } else if(timer.get() < 3) {
            if(m_arm.getUSDistance(false) <= 33) {
                x = 0; y = 0; rotation = 0; robotOrt = false; // -gyroPID.calculate(gyro,this.masterAngle + 179.9) 
            } else {
                x = 30; y = 0; rotation = -gyroPID.calculate(gyro, 0 /* this.masterAngle */); robotOrt = true;      
            }
            up = false;
            intake = -0.3;
            s_speed = 0;
        } else {
            up = true;
            c_speed = Conveyer.Speed.STOP;
            s_speed = 0;
            x = 0; y = 0; rotation = 0; robotOrt = false;
        }
        
        m_conveyer.setShooterSpeed(s_speed);
        m_conveyer.setBackShooterSpeed(sb_speed);
        m_arm.setDriveArmMotor(up);
        m_arm.setDriveArmIntakeMotor(intake);

        drive(x, y, rotation, robotOrt); 
    }

    private void auto4() {
        double x = 0;
        double y = 0;
        double rotation = 0;
        boolean robotOrt = false;
        float gyro = m_drivetrainSubsystem.getGyro();

        x = 0; y = 0; rotation = -gyroPID.calculate(gyro, 179.9); robotOrt = true;

        drive(x, y, rotation, robotOrt); 
    }

    private void auto5() {
        double x = 0;
        double y = 0;
        double rotation = 0;
        boolean robotOrt = false;
        float gyro = m_drivetrainSubsystem.getGyro();

        //3 = 135.5
        //1 = 45.2

        
        //x = 0; y = 0; rotation = -gyroPID.calculate(gyro, 179.9); robotOrt = true;

        if(timer.get() < 7) {
            x = -xPID.calculate(-135, -m_drivetrainSubsystem.getAverageEncoderInches()); y = 0; rotation = -gyroPID.calculate(gyro, 0 /* this.masterAngle */); robotOrt = true;
        } else if(timer.get() < 14) {
            x = -xPID.calculate(-4, -m_drivetrainSubsystem.getAverageEncoderInches()); y = 0; rotation = -gyroPID.calculate(gyro, 0 /* this.masterAngle */); robotOrt = true;
        }

       // return;

        /* if(seq == 0 && timer.get() > 7 && -m_drivetrainSubsystem.getAverageEncoderInches() < -135){
            nextSeq = 1; SeqToggle = true;
        } else if(seq == 2 && timer.get() > 21 && gyro > 180){
            nextSeq = 2; SeqToggle = true;
        } else if(seq == 3 && timer.get() > 28 && gyro > 9.45){
            nextSeq = 3; SeqToggle = true;
        } else if(seq == 4 && timer.get() > 34 && -m_drivetrainSubsystem.getAverageEncoderInches() < 100.93){
            nextSeq = 4; SeqToggle = true;
        } else if(seq == 5 && timer.get() > 42 && -m_drivetrainSubsystem.getAverageEncoderInches() < 37.16){
            nextSeq = 5; SeqToggle = true;
        } else if(seq == 6 && timer.get() > 49 && -m_drivetrainSubsystem.getAverageEncoderInches() < 157.14 && gyro < 180){
            nextSeq = 6; SeqToggle = true;
        } else if(seq == 7 && timer.get() > 56 && -m_drivetrainSubsystem.getAverageEncoderInches() < -157.14 ){
            nextSeq = 7; SeqToggle = true;
        } else if(seq == 8 && timer.get() > 63 && gyro < 39.22){
            nextSeq = 8; SeqToggle = true;
        }

        if(nextSeq != seq) {
            seq = -1;
        }

        if(SeqToggle){
            seq = -1;
        }
        

        switch (seq) {
            case 0:
                x = xPID.calculate(-51.58, m_drivetrainSubsystem.getAverageEncoderInches()); y = 0; rotation = -gyroPID.calculate(gyro, 0 ); robotOrt = true;
                break;
            case 1:
                x = 0; y = 0; rotation = -gyroPID.calculate(gyro, 180 ); robotOrt = true;
                break;
            case 2:
                x = 0; y = 0; rotation = -gyroPID.calculate(gyro, 9.45 ); robotOrt = true;
                break;
            case 3:
                x = 0; y = yPID.calculate(100.93, m_drivetrainSubsystem.getAverageEncoderInches()); rotation = 0; robotOrt = true;
                break;
            case 4:
                x = xPID.calculate(37.16, m_drivetrainSubsystem.getAverageEncoderInches()); y = 0; rotation = 0; robotOrt = true;
                break;
            case 5:
                x = 0; y = yPID.calculate(157.14, m_drivetrainSubsystem.getAverageEncoderInches()); rotation = -gyroPID.calculate(gyro, 180 ); robotOrt = true;  
                break;
            case 6:
                x = 0; y = yPID.calculate(-157.14, m_drivetrainSubsystem.getAverageEncoderInches()); rotation = 0; robotOrt = true;
                break;
            case 7:
                x = 0; y = 0; rotation = -gyroPID.calculate(gyro, 39.22 ); robotOrt = true;
                break;
            case -1: //reset
                x = 0; y = 0; rotation = 0; robotOrt = true;
                m_drivetrainSubsystem.zeroGyroscope();
                m_drivetrainSubsystem.resetDriveEncoders();
                break;
            default:
            x = 0; y = 0; rotation = 0; robotOrt = true;
                break;
        }

        if(nextSeq != seq) {
            seq = nextSeq;
        }

        if(SeqToggle){
            seq = nextSeq;
            SeqToggle = false;
        } */

        /* if(x > 0.2) {
            x = 0.2;
        } */

        if(x < (-0.2 * DrivetrainSubsystem.MaxVelocity) && x < 0) {
            x = (-0.2 * DrivetrainSubsystem.MaxVelocity);
        }

        if(x > (0.2 * DrivetrainSubsystem.MaxVelocity) && x > 0) {
            x = (0.2 * DrivetrainSubsystem.MaxVelocity);
        }

        drive(x, y, rotation, robotOrt); 
    }

    @Override
    public void execute() {

        //Select Autp
        //TODO Make sure this works!
        switch (m_chooser.getSelected()) {
            case 0:
                auto1();
                break;
            case 1:
                auto2();
                break;
            case 2:
                auto3();
                break;
            case 3:
            auto4();
                //auto4();
                break;
            default:
                return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setGyro(this.masterAngle + m_drivetrainSubsystem.getGyro());
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        timer.stop();
        timer.reset();
        as1.stop();
        as1.reset();
        m_conveyer.enableTeleop(true);
        m_arm.enableTeleop(true);
        m_arm.setDriveArmMotor(true);
        m_arm.setDriveArmIntakeMotor(0);
    }
}
