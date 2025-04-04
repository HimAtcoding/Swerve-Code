package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntake extends Command {
    private final IntakeSubsystem intake;
    private final double targetRotations;

    public MoveIntake(IntakeSubsystem intake, double targetRotations) {
        this.intake = intake;
        this.targetRotations = targetRotations;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setTargetRotations(targetRotations);
    }

    @Override
    public boolean isFinished() {
        return intake.atTarget(targetRotations, 0.1);
    }
}
