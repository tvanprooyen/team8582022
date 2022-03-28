
package frc.robot.commands;

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
    private PIDController gyroPID = new PIDController(1.5, 0, 0);  // 0.004
    private final Timer timer = new Timer();
    private final Timer as1 = new Timer();
    private final LimeLight limeLight = new LimeLight(-0.02f, 0.02f, //Target Steer PID
                                                        44.125, 101.75, 20); // Target Distance
    
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
            drive(-20, 0, -gyroPID.calculate(gyro, this.masterAngle) * DrivetrainSubsystem.MaxAngularVelocity, true);
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
        double minspeed = 3.75;
        double maxspeed = 5.04;
        double mindist = 100;
        double maxdist = 180;
        double slope = (maxspeed-minspeed)/(maxdist-mindist);
        double s_speed = (slope*m_conveyer.GetDistance()) - (slope*mindist) + minspeed;
        float gyro = m_drivetrainSubsystem.getGyro();
        boolean up = true;
        double intake = 0;
        double x = 0;
        double y = 0;
        double rotation = 0;
        boolean robotOrt = false;
        
        if(timer.get() < 3) {
            if(m_arm.getUSDistance(false) <= 33) {
                x = 0; y = 0; rotation = 0; robotOrt = false; // -gyroPID.calculate(gyro,this.masterAngle + 179.9) 
            } else {
                x = 30; y = 0; rotation = -gyroPID.calculate(gyro,this.masterAngle); robotOrt = true;      
            }
            up = false;
            intake = -0.3;
            s_speed = 0;
        } else if(timer.get() < 3.1) {
            up = true;
            intake = 0;
            s_speed = 0;
            x = 0; y = 0; rotation = 0; robotOrt = false;       
        } else if(timer.get() < 5) {
            up = true;
            intake = 0;
            s_speed = 0;
            //drive(0, 0, -gyroPID.calculate(gyro,this.masterAngle + 179.9), true); 
            x = 0; y = 0; rotation = -gyroPID.calculate(gyro,this.masterAngle - 179.9); robotOrt = true;
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
            x = 0; y = 0; rotation = 0; robotOrt = false;
        }
        
        m_conveyer.setShooterSpeed(s_speed);

        m_arm.setDriveArmMotor(up);
        m_arm.setDriveArmIntakeMotor(intake);

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
        
            default:
                return;
        }
    }

    @Override
    public void end(boolean interrupted) {
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
