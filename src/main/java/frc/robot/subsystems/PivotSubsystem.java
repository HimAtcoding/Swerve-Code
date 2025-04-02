package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor;

    public PivotSubsystem() {
        pivotMotor = new SparkMax(11, MotorType.kBrushless);
        pivotMotor.getEncoder().setPosition(0);
    }

    public SparkMax getPivotMotor() {
        return pivotMotor;
    }
}
