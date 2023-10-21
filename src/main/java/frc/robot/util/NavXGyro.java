// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class NavXGyro extends AHRS{

    private static NavXGyro instance;
    public static double zeroHeading;
    public static double zeroAngle;

    /** Creates a new NavXGyro. */
    private NavXGyro() {
        super(SPI.Port.kMXP);

        zeroHeading = getNavHeading();
        zeroAngle = getNavAngle();
        System.out.println("Setup ZeroAngle " + zeroAngle);

    }

    // Public Methods
    public static NavXGyro getInstance() {
        if (instance == null) {
            instance = new NavXGyro();
        }
        return instance;
    }

    public double getNavHeading() {
        double heading = getFusedHeading();
        return heading;
    }

    public double getNavAngle() {
        double angle = getAngle();
        return angle;
    }

    public void zeroNavHeading() {
        //navX.zeroYaw();
        reset();
        zeroHeading = getNavHeading();
        zeroAngle = getNavAngle();
        System.out.println("ZeroHeading: " + zeroHeading);
        System.out.println("ZeroAngle: " + zeroAngle);
    }

    public double getZeroHeading(){
        return zeroHeading;
    }

    public double getZeroAngle(){
        return zeroAngle;
    }

    public Rotation2d getNavXRotation2D(){
        return Rotation2d.fromDegrees(getAngle());
    }

    public double getHeading() {
        return Math.IEEEremainder(-getNavAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetGyro(){
        reset();
    }
    public void calibrateGyro(){
        calibrate();
    }

}
