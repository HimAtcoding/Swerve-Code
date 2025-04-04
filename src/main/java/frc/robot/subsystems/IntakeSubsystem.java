package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController; 

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(10, MotorType.kBrushless);
        encoder = intakeMotor.getEncoder();
        closedLoopController = intakeMotor.getClosedLoopController();

        encoder.setPosition(0);
        
    }

    public void setTargetRotations(double rotations) {
        closedLoopController.setReference(rotations, ControlType.kPosition);
    }

    public double getCurrentRotations() {
        return encoder.getPosition();
    }

    public boolean atTarget(double target, double tolerance) {
        return Math.abs(getCurrentRotations() - target) < tolerance;
    }

    @Override
    public void periodic() {}
}
