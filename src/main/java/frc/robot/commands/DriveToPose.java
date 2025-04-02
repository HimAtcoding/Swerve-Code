package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers;

public class DriveToPose extends Command {
    private final SwerveSubsystem swerve;
    private Pose2d targetPose;

    private final PIDController xController = new PIDController(100, 0, 0);
    private final PIDController yController = new PIDController(100, 0, 0);
    private final PIDController thetaController = new PIDController(3, 0, 0);

    public DriveToPose(SwerveSubsystem swerve) {
        this.swerve = swerve;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (!LimelightHelpers.getTV("limelight")) {
            System.out.println("[DriveToTagOffset] No tag detected — aborting.");
            cancel();
            return;
        }

        // Pose of tag relative to robot
        Pose3d tagRelToRobot = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

        // Invert it to get robot pose relative to tag
        Transform2d robotRelToTag = new Transform2d(
            new Translation2d(tagRelToRobot.getX(), tagRelToRobot.getY()),
            Rotation2d.fromDegrees(tagRelToRobot.getRotation().getZ())
        );

        // Define desired offset from tag (e.g., 1m in front of it)
        Transform2d desiredOffsetFromTag = new Transform2d(
            new Translation2d(0.0, 0.0), // 1 meter in front
            Rotation2d.fromDegrees(0)   // Face the tag
        );
       System.out.println(desiredOffsetFromTag);

        // Robot pose in field space = tag pose in field space ⊕ (robotRelToTag ⊕ offset)
        Pose2d currentPose = swerve.getPose(); // just odometry based pose, vision is not used
        // fix target pose to be tag pose transformed by offset, no reason to use the robot pose and add the uncertainty of that into the target pose
        // test with id 22
        targetPose = currentPose.transformBy(robotRelToTag).transformBy(desiredOffsetFromTag);

        System.out.println("[DriveToTagOffset] Target field-relative pose: " + targetPose);
    }

    @Override
    public void execute() {
        Pose2d current = swerve.getPose();
        

        double xSpeed = xController.calculate(current.getX(), targetPose.getX());
        double ySpeed = yController.calculate(current.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(
            current.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        swerve.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, true, false); // field-relative
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) return true;

        Pose2d current = swerve.getPose();
        boolean closeX = Math.abs(current.getX() - targetPose.getX()) < 0.01;
        boolean closeY = Math.abs(current.getY() - targetPose.getY()) < 0.01;
        boolean closeRot = Math.abs(
            current.getRotation().minus(targetPose.getRotation()).getDegrees()
        ) < 0.1;

        return closeX && closeY && closeRot;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
        System.out.println("[DriveToTagOffset] Finished or interrupted");
    }
}