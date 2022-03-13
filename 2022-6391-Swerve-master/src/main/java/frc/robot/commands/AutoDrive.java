
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDrive extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Conveyer m_conveyer;
    private final Timer timer = new Timer();
    private final Timer as1 = new Timer();

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
        double s_speed = (slope*GetDistance()) - (slope*mindist) + minspeed;
        /* if(timer.get() < 15) {
            drive(0, 0, 0);
        }*/ if(timer.get() < 2) {
            drive(-40, 0, 0);
            c_speed = Conveyer.Speed.STOP;
            s_speed = 0;
        } else if(timer.get() < 4) {

            double Kp = -0.07f;
            double min_command = 0.02f;

            double Rotation = 0;
            
            //Target
            double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            
            //PID
            double heading_error = -tx;
            double steering_adjust = 0.0f;
            if (tx > 1.0){
                    steering_adjust = Kp*heading_error - min_command;
            }else if (tx < 1.0){
                    steering_adjust = Kp*heading_error + min_command;
            }
            Rotation -= steering_adjust;

            drive(0, 0, Rotation * DrivetrainSubsystem.MaxVelocity);
            c_speed = Conveyer.Speed.STOP;
            s_speed = 0;
        }else if(timer.get() < 4.5) {
            //Sequence #1
            drive(0, 0, 0);
            s_speed = (slope*GetDistance()) - (slope*mindist) + minspeed;
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

    public double GetDistance(){
        double distance;

        double a2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        distance = (101.75-44.125)/(Math.tan((Math.PI/180)*(20+a2)));

       return distance;
    }

    public double trackTarget(double m_axis) {
        //Constants
        //double getSteerConstant = 0;
        double Kp = -0.085f;
        double min_command = 0.02f;

        double joyAxisAvg = Math.abs(m_axis);

        /* if(driver1.getRawAxis(0) > 0.1 || driver1.getRawAxis(1) > 0.1 || driver1.getRawAxis(4) > 0.1) {
            Kp = -0.085f;
        } */
         
        //Target
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
         
        //PID
        double heading_error = -tx;
        double steering_adjust = 0.0f;
        if (tx > 1.0){
                steering_adjust = Kp*heading_error - min_command;
        }else if (tx < 1.0){
                steering_adjust = Kp*heading_error + min_command;
        }

        //Post to Dashboard

        //Default to JoyStick
        double Rotation = m_axis;

        Rotation -= steering_adjust;

        return Rotation;
    }
}
