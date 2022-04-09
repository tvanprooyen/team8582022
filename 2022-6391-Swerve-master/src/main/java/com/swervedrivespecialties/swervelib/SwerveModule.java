package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    double getDrivePosistion();

    void resetDriveEncoders();

    void set(double driveVoltage, double steerAngle);
}
