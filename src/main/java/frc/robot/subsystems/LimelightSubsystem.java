package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1.0;
    }

    public Pose2d getBotPose2d() {
        double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
        double x = botpose[0]; // meters
        double y = botpose[1]; // meters
        double rotationDeg = botpose[5];
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDeg));
    }
}