package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team7520.robot.subsystems.arm.Hand;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HandCommand extends CommandBase {
    private final Hand handSubsystem;

    private final DoubleSupplier speed;

    public HandCommand(Hand handSubsystem, DoubleSupplier speed) {
        this.handSubsystem = handSubsystem;
        this.speed = speed;
        addRequirements(handSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        handSubsystem.left.set(speed.getAsDouble());
        handSubsystem.right.set(speed.getAsDouble());
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
