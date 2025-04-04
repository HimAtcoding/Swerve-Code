package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToAngleCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private final double targetRotations;
    private final double tolerance = 0.5; // Adjust as needed

    public PivotToAngleCommand(PivotSubsystem pivotSubsystem, double targetRotations) {
        this.pivotSubsystem = pivotSubsystem;
        this.targetRotations = targetRotations;
        addRequirements(pivotSubsystem);

    
    }

    @Override
    public void initialize() {
        pivotSubsystem.setPivotPosition(targetRotations);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivotSubsystem.getEncoderPosition() - targetRotations) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pivotSubsystem.setPivotPosition(pivotSubsystem.getEncoderPosition()); // Stop at current position
        }
    }
}