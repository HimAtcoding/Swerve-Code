package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForwardDistance extends Command {
    private final SwerveSubsystem swerve;
    private final double forwardMeters, rightMeters;
    private final Rotation2d targetHeading;
    private Pose2d targetPose;

    public final PIDController xController = new PIDController(1.5, 0, 0);
    public final PIDController yController = new PIDController(1.5, 0, 0);
    public final PIDController thetaController = new PIDController(3.0, 0, 0);

    public DriveForwardDistance(SwerveSubsystem swerve, double forwardMeters, double rightMeters, Rotation2d targetHeading) {
        this.swerve = swerve;
        this.forwardMeters = forwardMeters;
        this.rightMeters = rightMeters;
        this.targetHeading = targetHeading;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        try {
            Pose2d currentPose = swerve.getPose();
            System.out.println("Current pose: " + currentPose);

            Translation2d targetTranslation = currentPose.getTranslation()
                .plus(new Translation2d(rightMeters, forwardMeters));

            targetPose = new Pose2d(targetTranslation, targetHeading);
            System.out.println("Target pose: " + targetPose);

            xController.reset();
            yController.reset();
            thetaController.reset();
        } catch (Exception e) {
            DriverStation.reportError("DriveToPose init failed: " + e.getMessage(), e.getStackTrace());
        }
    }

    @Override
    public void execute() {
        Pose2d current = swerve.getPose();

        double xSpeed = xController.calculate(current.getX(), targetPose.getX());
        double ySpeed = yController.calculate(current.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Translation2d translation = new Translation2d(xSpeed, ySpeed);
        swerve.drive(translation, thetaSpeed, true, true);
    }

    @Override
    public boolean isFinished() {
        Pose2d current = swerve.getPose();
        System.out.println("Current Pose: " + current);
        System.out.println("Target Pose: " + targetPose);
    
        boolean posDone = 
            Math.abs(current.getX() - targetPose.getX()) < 0.05 &&
            Math.abs(current.getY() - targetPose.getY()) < 0.05;
    
        boolean rotDone =
            Math.abs(current.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 3.0;
    
        System.out.println("posDone: " + posDone + ", rotDone: " + rotDone);
    
        return posDone;
        // return posDone && rotDone;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    

}