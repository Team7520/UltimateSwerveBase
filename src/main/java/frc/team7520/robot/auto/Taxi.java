package frc.team7520.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team7520.robot.subsystems.swerve.AutoBase;
import frc.team7520.robot.subsystems.swerve.SwerveBase;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import frc.team7520.robot.util.NavXGyro;

public class Taxi extends AutoBase {
    public Taxi(SwerveBase swerve) {
        super(swerve);
        PathPlannerTrajectory taxi = PathPlanner.loadPath("taxi", 4.0, 3.0, true);
        PPSwerveControllerCommand command = baseSwerveCommand(taxi);
        PathPlannerState initialState = taxi.getInitialState();

        addCommands(
                new InstantCommand(() -> NavXGyro.getInstance().resetGyro()),
                new InstantCommand(
                        () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                                initialState.holonomicRotation))),
                command);

    }


}
