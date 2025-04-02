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

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    public ElevatorSubsystem(int motorID) {
        elevatorMotor = new SparkMax(motorID, MotorType.kBrushless);
        encoder = elevatorMotor.getEncoder();
        closedLoopController = elevatorMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(.1, 0, 0);
        closedLoopConfig.outputRange(-1, 1);

        config.apply(closedLoopConfig);

        elevatorMotor.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

        encoder.setPosition(0);
    }

    public void setElevatorPosition(double targetRotations) {
        closedLoopController.setReference(targetRotations, ControlType.kPosition);
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }
}