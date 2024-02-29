package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    private Pose2d tmpEndPose = null;

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
    private final Command m_driveSmartPosition = Commands.runOnce(()->s_Swerve.setSmartPosition());
    private final Command m_driveSmartDirection = Commands.runOnce(()->s_Swerve.setSmartAngle(90));
    private final Command m_reset = Commands.runOnce(()->s_Swerve.resetOdometry(new Pose2d()));
    private final Command m_preplanned = Commands.runOnce(()->pathplanner_preplanned());
    private final Command m_apriltag_1Steps = Commands.runOnce(()->{
        Pose2d photonPose = GetPhotonPose2d();
        if (photonPose != null)
        {
            var cmd = goToPose_photon(photonPose);
            if (cmd != null)
            {
                CommandScheduler.getInstance().schedule(cmd);
            }
        }
    });
    private final Command m_apriltag_2Steps = 
        Commands.runOnce(()->
        {
            //CommandScheduler.getInstance().cancelAll();
            tmpEndPose = null;
            Pose2d photonPose = GetPhotonPose2d();
            if (photonPose != null)
            {
                var cmd = goToPoseCommand();
                if (cmd != null)
                {
                    CommandScheduler.getInstance().schedule(cmd);
                }
            }
        });

    private final Command m_gotoPose_photon_withDefaultPose = 
        Commands.runOnce(()->
        {
            //var cmd1 = new CameraGoToPoseCommand(s_Swerve);
            //* 

            Pose2d photonPose = GetPhotonPose2d();
            if (photonPose == null)
                photonPose = tmpEndPose;
            if (photonPose != null)
            {
                Pose2d endPose = photonPose;
                SmartDashboard.putNumber("photonPose second step end_x", endPose.getX());
                SmartDashboard.putNumber("photonPose second step end_y", endPose.getY());
                SmartDashboard.putNumber("photonPose second step end_yaw", endPose.getRotation().getDegrees());

                var cmd = new SequentialCommandGroup(
                    goToPose_photon(endPose), 
                    goToPose(endPose),
                    goToPose(endPose),
                    goToPose(endPose)
                    );
                //var cmd = goToPose_photon(endPose);
                CommandScheduler.getInstance().schedule(cmd);
            }
            //*/
        });

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
       // SmartDashboard.putBoolean("auto driving", false);gy
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> driver.getRawButtonPressed(XboxController.Button.kY.value),
                () -> true
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

        YButton.onTrue(m_preplanned);
        XButton.onTrue(m_apriltag_2Steps);
        //XButton.onTrue(m_apriltag_1Steps);
        
        LeftBumperButton.onTrue(m_reset);

        // Manual method
        //StartButton.onTrue(m_driveSmartDirection);
        //BackButton.onTrue(m_driveSmartPosition);
        StartButton.onTrue(m_driveSmartPositionPoint);
        BackButton.onTrue(m_driveHeading);

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

    public void pathplanner_preplanned()
    {
        Command cmd1 = goToPoseCommand_preplanned(); 
        Command cmd2 = goToPoseCommand_preplanned(); 
        Command cmd3 = goToPoseCommand_preplanned(); 
        Command cmd4 = goToPoseCommand_preplanned(); 
        Command cmd5 = goToPoseCommand_preplanned(); 
        Command cmd6 = goToPoseCommand_preplanned(); 
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup( cmd1, cmd2, cmd3, cmd4, cmd5, cmd6)
        );
    } 

    public Command goToPoseCommand_preplanned()
    {
        //s_Swerve.resetOdometry(s_Swerve.getPose());
        PathPlannerPath path = PathPlannerPath.fromPathFile(
            //"Example Path"
            //"straight line x"
            // "straight line y"
            //"turn 90"
            "Path Rotation Target"
            //"turn big 90"
            );
        //path.preventFlipping =true;
        var points = path.getAllPathPoints();
        var lastP = points.get(points.size() - 1).position;
        var curPose = s_Swerve.getPose().getTranslation();
        var lastAngle = path.getGoalEndState().getRotation().getDegrees();
        var curAngle = s_Swerve.getPose().getRotation().getDegrees();
        if (curPose.getDistance(lastP) < 0.1)
        {
            if (Math.abs(curAngle - lastAngle) < 2)
                return null;
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    private Pose2d GetPhotonPose2d()
    {
        var photonPose = s_Swerve.GetPhotonvisionPose2d();
        photonPose = ConvertToAbsolutePose(photonPose);
        
        SmartDashboard.putBoolean("photonPose found ", (photonPose != null));
        return photonPose;
    }

    private Pose2d ConvertToAbsolutePose(Pose2d srcPose)
    {
        if (srcPose == null) return null;

        var deltaPose = srcPose;
        Pose2d startPose = s_Swerve.getPose();

        Translation2d startTranslation2d = startPose.getTranslation();
        Translation2d deltaTranslation2d = deltaPose.getTranslation();
        Translation2d endTranslation2d = startTranslation2d.plus(deltaTranslation2d);
        Rotation2d starRotation2d = startPose.getRotation();
        Rotation2d deltaRotation2d = deltaPose.getRotation();
        Rotation2d endRotation2d = starRotation2d.plus(deltaRotation2d);

        Pose2d endPose = new Pose2d(endTranslation2d,endRotation2d);

        return endPose;
    }



    public Command goToPoseCommand()
    {
        var photonPose = GetPhotonPose2d();
        if (photonPose == null)
            return null;

        // output first photonview endpose
        SmartDashboard.putNumber("photonPose first step end_x", photonPose.getX());
        SmartDashboard.putNumber("photonPose first step end_y", photonPose.getY());
        SmartDashboard.putNumber("photonPose first step end_yaw", photonPose.getRotation().getDegrees());

        // clear second photonview endpose
        SmartDashboard.putNumber("photonPose second step end_x", 0);
        SmartDashboard.putNumber("photonPose second step end_y", 0);
        SmartDashboard.putNumber("photonPose second step end_yaw", 0);

        Pose2d endPose = photonPose;

        var startPose = s_Swerve.getPose();
        SmartDashboard.putNumber("from_x", startPose.getX());
        SmartDashboard.putNumber("from_y", startPose.getY());
        SmartDashboard.putNumber("from_yaw", startPose.getRotation().getDegrees());

        SmartDashboard.putNumber("gotoP_x", endPose.getX());
        SmartDashboard.putNumber("gotoP_y", endPose.getY());
        SmartDashboard.putNumber("gotoP_yaw", endPose.getRotation().getDegrees());

        return goToPose_photon_2Steps(endPose);
    }

    public Command goToPose_photon_2Steps(Pose2d endPose)
    {
        try
        {
            // get the endPose from photonvision
            // if it very close, go there directly
            // otherwise, go to the middle pose first, check photovision again, 
            // get the new endPose, then go to the final endPose
            Pose2d startPose = s_Swerve.getPose();
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                startPose,
                endPose
            );
            double lengthToCompare = 1;
            int size = bezierPoints.size();
            var totalDistance = getTotalDistance(bezierPoints);
            if (totalDistance < lengthToCompare || size < 3)
            {
                /*
                return new SequentialCommandGroup(
                    goToPose_photon(endPose),
                    goToPose(endPose),
                    goToPose(endPose),
                    goToPose(endPose)
                );
                 */
                return goToPose_photon(endPose);
            }
            else
            {
                double distance = 0; 
                Translation2d midT = bezierPoints.get(0);
                for(int i=1; i<size; i++)
                {
                    distance += bezierPoints.get(i).getDistance(bezierPoints.get(i-1));
                    midT = bezierPoints.get(i);
                    if(totalDistance - distance <= lengthToCompare)
                        break;
                }
                Pose2d midPose = new Pose2d(midT, endPose.getRotation());
                SmartDashboard.putNumber("midPose_x", midPose.getX());
                SmartDashboard.putNumber("midPose_y", midPose.getY());
                SmartDashboard.putNumber("midPose_yaw", midPose.getRotation().getDegrees());
                tmpEndPose = endPose;

                var cmd = goToPose_photon(midPose);
                if (cmd != null)
                {
                    return new SequentialCommandGroup(
                        cmd,
                        //goToPose(midPose),
                        m_gotoPose_photon_withDefaultPose
                    );
                }
                else
                    return null;
            }
        }
        catch(Exception ex)
        {
            System.out.println("ERROR: " + ex.getMessage());
            return null;
        }
    }

    public Command goToPose_photon(Pose2d endPose)
    {
        // go to the endPose directly, finish cart heading rotation in the middle
        Pose2d startPose = s_Swerve.getPose();
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose,
            endPose
        );
        
        RotationTarget rt = new RotationTarget(
            0.5, 
            endPose.getRotation(), false);
        List<RotationTarget> rts = Arrays.asList(rt);
        List<ConstraintsZone> czs = Arrays.asList();
        List<EventMarker> ems = Arrays.asList();
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                rts,
                czs, // list ConstraintsZone
                ems, // list event marker
                new PathConstraints(0.4, 1, Math.PI,  Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, endPose.getRotation()), // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
                false
        );
        
        path.preventFlipping =true;
        
        return AutoBuilder.followPath(path);
    }

    public Command goToPose(Pose2d endPose)
    {
        // go to the endPose directly
        double endAngle=0;
        //endAngle = SmartDashboard.getNumber("endAngle", 0);
        Pose2d startPose = s_Swerve.getPose();
        endAngle = endPose.getRotation().getDegrees();


        var lastP = endPose.getTranslation();
        var curPose = s_Swerve.getPose().getTranslation();
        var lastAngle = endPose.getRotation().getDegrees();
        var curAngle = s_Swerve.getPose().getRotation().getDegrees();
        if (curPose.getDistance(lastP) < 0.05)
        {
            if (Math.abs(curAngle - lastAngle) < 1)
                return null;
        }

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose,
            endPose
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(0.4, 1, Math.PI,  Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(endAngle)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        
        return AutoBuilder.followPath(path);
    }


    public static double getTotalDistance(List<Translation2d> waypoints) {
        double totalDistance = 0.0;

        for (int i = 1; i < waypoints.size(); i++) {
            Translation2d currentPoint = waypoints.get(i);
            Translation2d prevPoint = waypoints.get(i - 1);

            double distance = currentPoint.getDistance(prevPoint);
            totalDistance += distance;
        }

        return totalDistance;
    }
}
