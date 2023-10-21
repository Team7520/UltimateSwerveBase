package frc.team7520.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team7520.robot.subsystems.arm.Hand;

import java.util.function.DoubleSupplier;

public class HandCommand extends CommandBase {
    private final Hand handSubsystem;

    private final DoubleSupplier speedSup;

    public HandCommand(Hand handSubsystem, DoubleSupplier speedSup) {
        this.handSubsystem = handSubsystem;
        this.speedSup = speedSup;
        addRequirements(handSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speedVal = speedSup.getAsDouble();
        if(speedVal > 0){
            handSubsystem.left.setIdleMode(CANSparkMax.IdleMode.kCoast);
            handSubsystem.right.setIdleMode(CANSparkMax.IdleMode.kCoast);
            handSubsystem.left.set(speedVal*0.03);
            handSubsystem.right.set(speedVal*0.03);
        }else {
            handSubsystem.left.setIdleMode(CANSparkMax.IdleMode.kBrake);
            handSubsystem.right.setIdleMode(CANSparkMax.IdleMode.kBrake);
            handSubsystem.left.set((speedVal*0.3)-0.03);
            handSubsystem.right.set((speedVal*0.3)-0.03);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
