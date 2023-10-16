package frc.team7520.robot.subsystems.arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hand extends SubsystemBase {

    public CANSparkMax left = new CANSparkMax(22, CANSparkMaxLowLevel.MotorType.kBrushless);
    public CANSparkMax right = new CANSparkMax(23, CANSparkMaxLowLevel.MotorType.kBrushless);

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this CubeHand. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static Hand INSTANCE = new Hand();

    /**
     * Returns the Singleton instance of this CubeHand. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code CubeHand.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Hand getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this CubeHand. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private Hand() {
    }

    @Override
    public void periodic(){
//        SmartDashboard.putNumber("handPose", wrist.getEncoder().getPosition());
    }

    public InstantCommand intake(){
        return new InstantCommand(
                () -> {
                    left.set(-0.2);
                    right.set(-0.2);
                }
        );
    }

    public InstantCommand shoot(){
        return new InstantCommand(
                () -> {
                    left.set(0.5);
                    right.set(0.5);
                }
        );
    }

    public InstantCommand stop(){
        return new InstantCommand(
                () -> {
                    left.set(0);
                    right.set(0);
                }
        );
    }
}

