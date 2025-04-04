package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    public PivotSubsystem(int motorID) {
        pivotMotor = new SparkMax(motorID, MotorType.kBrushless);
        encoder = pivotMotor.getEncoder();
        closedLoopController = pivotMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(.1, 0, 0);
        closedLoopConfig.outputRange(-1, 1);

        config.apply(closedLoopConfig);

        pivotMotor.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

        encoder.setPosition(0);
    }

    public void setPivotPosition(double targetRotations) {
        closedLoopController.setReference(targetRotations, ControlType.kPosition);
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }
}