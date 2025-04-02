package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorToPosition extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double targetRotations;
    private final double tolerance = 0.5; // Adjust as needed

    public MoveElevatorToPosition(ElevatorSubsystem elevatorSubsystem, double targetRotations) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetRotations = targetRotations;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorPosition(targetRotations);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubsystem.getEncoderPosition() - targetRotations) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevatorSubsystem.setElevatorPosition(elevatorSubsystem.getEncoderPosition()); // Stop at current position
        }
    }
}