// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.logging;
import frc.lib.util.loggingUtil.LogManager;
// import frc.lib.util.loggingUtil.LogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveBase;

public class LoggingSubsystem extends SubsystemBase {

    private final SwerveBase s_Swerve;

    /** Creates a new LoggingSubsystem. */
  public LoggingSubsystem(SwerveBase s_Swerve) {

    this.s_Swerve = s_Swerve;
  }
  public void updateSwerveLogs() {
    double[] actualStates = {
      s_Swerve.swerveMods[0].getAngle().getDegrees(),
      s_Swerve.swerveMods[0].getState().speedMetersPerSecond,
      s_Swerve.swerveMods[1].getAngle().getDegrees(),
      s_Swerve.swerveMods[1].getState().speedMetersPerSecond,
      s_Swerve.swerveMods[2].getAngle().getDegrees(),
      s_Swerve.swerveMods[2].getState().speedMetersPerSecond,
      s_Swerve.swerveMods[3].getAngle().getDegrees(),
      s_Swerve.swerveMods[3].getState().speedMetersPerSecond
    };
    LogManager.addDoubleArray("Swerve/actual swerve states", actualStates);
    double[] desiredStates = {
      s_Swerve.swerveMods[0].desiredState.angle.getDegrees(),
      s_Swerve.swerveMods[0].desiredState.speedMetersPerSecond,
    };
    LogManager.addDoubleArray("Swerve/Desired Swerve States", desiredStates);
  }

  @Override
  public void periodic() {
    updateSwerveLogs();

  }
}
