
package frc.robot.commands;

import com.team858.control.LimeLight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDrive extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Conveyer m_conveyer;
    private final Timer timer = new Timer();
    private final Timer as1 = new Timer();
    private final LimeLight limeLight = new LimeLight(-0.07f, 0.02f, //Target Steer PID
                                                        44.125, 101.75, 20); // Target Distance

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
                               Conveyer conveyer) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_conveyer = conveyer;
        addRequirements(drivetrainSubsystem);
        addRequirements(conveyer);
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

    @Override
    public void initialize() {
        timer.start();
        super.initialize();
    }

    @Override
    public void execute() {
        Conveyer.Speed c_speed = Conveyer.Speed.STOP;

        //linear line
        double minspeed = 3.6;
        double maxspeed = 5.04;
        double mindist = 100;
        double maxdist = 180;
        double slope = (maxspeed-minspeed)/(maxdist-mindist);
        double s_speed = (slope*limeLight.getDistance()) - (slope*mindist) + minspeed;


        if(timer.get() < 2) {
            drive(-40, 0, 0);
            c_speed = Conveyer.Speed.STOP;
            s_speed = 0;
        } else if(timer.get() < 4) {

            drive(0, 0, limeLight.trackTarget(0) * DrivetrainSubsystem.MaxVelocity);
            c_speed = Conveyer.Speed.STOP;
            s_speed = 0;
        } else if(timer.get() < 4.5) {
            //Sequence #1
            drive(0, 0, 0);
            s_speed = (slope*limeLight.getDistance()) - (slope*mindist) + minspeed;
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

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        timer.stop();
        timer.reset();
        as1.stop();
        as1.reset();
    }
}
