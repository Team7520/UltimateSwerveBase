package frc.team7520.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.team7520.robot.Robot;
import frc.team7520.robot.auto.Taxi;
import frc.team7520.robot.commands.*;
import frc.team7520.robot.subsystems.arm.Hand;
import frc.team7520.robot.subsystems.swerve.SwerveBase;
import frc.team7520.robot.subsystems.arm.Arm;

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

    /* Operator Controls */
    private final Joystick operator = new Joystick(1);

    JoystickButton operatorYButton = new JoystickButton(operator, XboxController.Button.kY.value);
    JoystickButton operatorAButton = new JoystickButton(operator, XboxController.Button.kA.value);
    JoystickButton operatorBButton = new JoystickButton(operator, XboxController.Button.kB.value);
    JoystickButton operatorXButton = new JoystickButton(operator, XboxController.Button.kX.value);



    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton autoMove = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final SwerveBase s_Swerve = new SwerveBase();
    private final Hand hand = Hand.getInstance();

    /* Commands */

    //example of auto move
    DriveToPoseCommand autoMoveCommand = new DriveToPoseCommand(
            s_Swerve,
            s_Swerve::getPose,
            new Pose2d(15.01, 1.52, new Rotation2d(0)),
            false
    );

    HandCommand handCommand = new HandCommand(
            hand,
            () -> {
                return (operator.getRawButton(XboxController.Button.kRightBumper.value) ? -1 : 1) * operator.getRawAxis(XboxController.Axis.kRightTrigger.value);
            }
    );

    /* Network Tables Elements */

    SendableChooser<Command> movementChooser = new SendableChooser<Command>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        SmartDashboard.putBoolean("auto driving", false);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> false,
                () -> false
            )
        );
        hand.setDefaultCommand(handCommand);
        /* Auto */
        PathPlannerServer.startServer(5811);
        movementChooser.setDefaultOption("taxi", new Taxi(s_Swerve));
        movementChooser.addOption("No Movement", new InstantCommand());
        SmartDashboard.putData("Movement", movementChooser);

        /* Networking */
        PortForwarder.add(5800, "10.75.20.40", 5800);
        PortForwarder.add(1181, "10.75.20.40", 1181);

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

        //example of auto move
        autoMove.whileTrue(autoMoveCommand);
        autoMove.toggleOnFalse(new InstantCommand(() -> autoMoveCommand.cancel()));

        Arm.getInstance().setDefaultCommand(Arm.getInstance().changePos(
                () -> operator.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> operator.getRawAxis(XboxController.Axis.kRightY.value)
        ));

        operatorYButton.whileTrue(Arm.getInstance().rest());
        operatorAButton.whileTrue(Arm.getInstance().cone());
        operatorBButton.whileTrue(Arm.getInstance().cube());
        operatorXButton.whileTrue(Arm.getInstance().floor());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return movementChooser.getSelected();
    }
}
