package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotToAngleCommand extends Command {

    private final SparkMax pivotMotor;
    private final RelativeEncoder encoder;
    private final double targetAngle;

    public PivotToAngleCommand(SparkMax pivotMotor, double targetAngle) {
        this.pivotMotor = pivotMotor;
        this.encoder = pivotMotor.getEncoder();
        this.targetAngle = targetAngle;

        addRequirements(); // Add subsystem if applicable
    }

    @Override
    public void initialize() {
        double gearRatio = 15.0;
        double motorRotations = (targetAngle / 360.0) * gearRatio;
        System.out.println("Target Angle: " + targetAngle + " degrees | Target Rotations: " + motorRotations);
    }

    @Override
    public void execute() {
        double gearRatio = 15.0;
        double motorRotations = (targetAngle / 360.0) * gearRatio;
        double currentPosition = encoder.getPosition();

        double power = (motorRotations - currentPosition) * 5; // P-controller
        pivotMotor.set(Math.max(-0.5, Math.min(0.5, power))); // Clamp power
    }

    @Override
    public boolean isFinished() {
        double gearRatio = 15.0;
        double motorRotations = (targetAngle / 360.0) * gearRatio;
        return Math.abs(encoder.getPosition() - motorRotations) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        pivotMotor.set(0);
        System.out.println("Pivot movement complete.");
    }
}
