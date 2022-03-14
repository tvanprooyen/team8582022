package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    double getDrivePosistion();

    void set(double driveVoltage, double steerAngle);
}
