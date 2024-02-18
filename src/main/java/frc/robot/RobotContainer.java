package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

//import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.swerve.SwerveBase;
import pabeles.concurrency.ConcurrencyOps.Reset;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton autoMove = new JoystickButton(driver, XboxController.Button.kB.value);

    
    //private final JoystickButton cameraDriveMove = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton angleDriveMove = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton XButton    = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton YButton    = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton StartButton    = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton BackButton    = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton LeftBumperButton    = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    

    /* Subsystems */
    private final SwerveBase s_Swerve = new SwerveBase();

    /* Commands */

        // Testing Use of Translation2d and Rotation2d with FieldOrientation
        private Translation2d velocity = new Translation2d(0.5, new Rotation2d(Math.PI*(0)));
        private Rotation2d angularVelocity = new Rotation2d(Math.PI/2);
        private Rotation2d angleOfRobot = new Rotation2d(0); // Angle of the robot must be dependent on the gyro's angle to be field oriented at all times
        private boolean fieldOriented = true;

    private Translation2d spot1 = new Translation2d();
    private Translation2d spot2 = new Translation2d(1, 1);
    /* Robin */
    // Use Translation2D & Rotation 2D and Kinematics method to calculation the movement
    private final Command m_driveSmartPositionPoint = Commands.runOnce(()->s_Swerve.setSmartPositionPoint(spot2, spot1, 1, new Rotation2d()));
    private final Command m_driveSpeed = Commands.runOnce(()->s_Swerve.setDriveSpeed(velocity, angularVelocity, angleOfRobot, fieldOriented));
    private final Command m_driveHeading = Commands.runOnce(()->s_Swerve.setDriveHeading(new Rotation2d(Math.PI/4)));
    /* Yan Hongtao */
    // Use manual calculate input for movement
    private final Command m_driveSmartPosition = Commands.runOnce(()->s_Swerve.setSmartAngle(90));
    private final Command m_driveSmartDirection = Commands.runOnce(()->s_Swerve.setSmartPosition());
    private final Command m_reset = Commands.runOnce(()->s_Swerve.resetOdometry(new Pose2d()));




    //example of auto move
    DriveToPoseCommand autoMoveCommand = new DriveToPoseCommand(
            s_Swerve,
            s_Swerve::getPose,
            new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
            false
    );

    /* Network Tables Elements */

    SendableChooser<Command> movementChooser = new SendableChooser<Command>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
       // SmartDashboard.putBoolean("auto driving", false);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
                () -> driver.getRawButtonPressed(XboxController.Button.kY.value),
                () -> false
            )
        );
        /* Auto */
        //PathPlannerServer.startServer(5811);
        //movementChooser.setDefaultOption("taxi", new Taxi(s_Swerve));
        movementChooser.addOption("No Movement", new InstantCommand());
        //SmartDashboard.putData("Movement", movementChooser);

        /* Networking */
        PortForwarder.add(5800, "10.75.20.40", 5800);
        PortForwarder.add(1181, "10.75.20.40", 1181);

        initializeAutoBuilder();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //cameraDriveMove.onTrue(new CameraDriveCommand(s_Swerve, s_Swerve::getPose));
        //angleDriveMove.onTrue(new AngleDriveCommand(s_Swerve, s_Swerve::getPose));

        // Kinematics method
        XButton.onTrue(m_driveHeading);
        //YButton.onTrue(m_driveSmartPositionPoint);

        YButton.onTrue(goToPoseCommand_preplanned());
        LeftBumperButton.onTrue(m_reset);

        // Manual method
        StartButton.onTrue(m_driveSmartDirection);
        BackButton.onTrue(m_driveSmartPosition);

        //example of auto move
        autoMove.whileTrue(autoMoveCommand);
        //autoMove.toggleOnFalse(new InstantCommand(() -> autoMoveCommand.cancel()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return movementChooser.getSelected();
    }

    public SwerveBase getSwerveBase() {
        return s_Swerve;
    }



    public void initializeAutoBuilder(){
        AutoBuilder.configureHolonomic(
            ()->s_Swerve.getPose(),
            (pose) -> {s_Swerve.resetOdometry(pose);},
            ()->s_Swerve.getChassisSpeeds(),
            (chassisSpeeds) -> {s_Swerve.setChassisSpeeds(chassisSpeeds,false);},
            AutoConstants.config,
            getAllianceColorBooleanSupplier(),
            s_Swerve
        );
    }

   public static BooleanSupplier getAllianceColorBooleanSupplier(){
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
  }

    public Command goToPoseCommand_preplanned()
    {
        //s_Swerve.resetOdometry(s_Swerve.getPose());
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        //path.preventFlipping =true;

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    public Command goToPoseCommand()
    {
        //Pose2d startPose = s_Swerve.getPose();
        Pose2d startPose = new Pose2d();
        Pose2d endPose = new Pose2d(
            new Translation2d(
                0, 1
            ),
            //startPose.getRotation().plus(Rotation2d.fromDegrees(90))
            //startPose.getRotation()
            Rotation2d.fromDegrees(0)
        );
        /*
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );
         */
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                startPose,
                endPose
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
}
