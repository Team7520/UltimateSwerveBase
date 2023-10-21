package frc.team7520.robot.commands;

import static frc.team7520.robot.Constants.AutoConstants.*;
import static frc.team7520.robot.Constants.Swerve.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.subsystems.swerve.SwerveBase;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {

    private static final double TRANSLATION_TOLERANCE = 0.005;
    private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
    private static final double FIELD_WIDTH_METERS = 8.0137;

    /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
    private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
            maxSpeed * 0.5,
            maxSpeed);
    private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            maxAngularVelocity * 0.4,
            maxAngularVelocity);

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private final SwerveBase drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final Pose2d goalPose;
    private final boolean useAllianceColor;

    public DriveToPoseCommand(
            SwerveBase drivetrainSubsystem,
            Supplier<Pose2d> poseProvider,
            Pose2d goalPose,
            boolean useAllianceColor) {
        this(drivetrainSubsystem, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, useAllianceColor);
    }

    public DriveToPoseCommand(
            SwerveBase drivetrainSubsystem,
            Supplier<Pose2d> poseProvider,
            Pose2d goalPose,
            TrapezoidProfile.Constraints xyConstraints,
            TrapezoidProfile.Constraints omegaConstraints,
            boolean useAllianceColor) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;
        this.goalPose = goalPose;
        this.useAllianceColor = useAllianceColor;
//
//        xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
//        yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);

        xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
        yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);

        xController.setTolerance(TRANSLATION_TOLERANCE);
        yController.setTolerance(TRANSLATION_TOLERANCE);
        thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(THETA_TOLERANCE);

        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void initialize() {
        resetPIDControllers();
        var pose = goalPose;
        if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());

            Rotation2d transformedHeading = pose.getRotation().times(-1);
            pose = new Pose2d(transformedTranslation, transformedHeading);
        }
        thetaController.setGoal(pose.getRotation().getRadians());
        xController.setGoal(pose.getX());
        yController.setGoal(pose.getY());  }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    private void resetPIDControllers() {
        var robotPose = poseProvider.get();
        thetaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("auto driving", true);
        var robotPose = poseProvider.get();
        // Drive to the goal
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
        if (thetaController.atGoal()) {
            omegaSpeed = 0;
        }

        drivetrainSubsystem.setModuleStates(swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation())));

    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("auto driving", false);
        drivetrainSubsystem.stop();
    }

}
