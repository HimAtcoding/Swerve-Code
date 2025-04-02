package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;

public class MoveToPoseCommand extends Command {
    private final SwerveSubsystem swerve;
    private Pose2d targetPose;
    private final HolonomicDriveController controller;
    private final Timer timer = new Timer();
    private Trajectory trajectory;

    public MoveToPoseCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.controller = null;
        // swerve.getSwerveDrive().addVisionMeasurement(LimelightHelpers.getBotPose2d(""), Timer.getFPGATimestamp());
        // double x = NetworkTableInstance.getDefault().getTable("limelight").getValue("tx").getDouble();
        // double y = NetworkTableInstance.getDefault().getTable("limelight").getValue("ty").getDouble();
        // double a = NetworkTableInstance.getDefault().getTable("limelight").getValue("ta").getDouble();
        // this.targetPose = new Pose2d(x, y, new Rotation2d(a));
        

        addRequirements(swerve);

        // PIDController xController = new PIDController(1.0, 0, 0);
        // PIDController yController = new PIDController(1.0, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         1.0, 0, 0, new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
        //         Math.PI, Math.PI / 2)
        // );
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // controller = new HolonomicDriveController(xController, yController, thetaController);
    }

    @Override
    public void initialize() {
        // if (targetPose != null) {
        // timer.reset();
        // timer.start();

        // // Generate a trajectory from current pose to target pose
        // TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);
        // trajectory = TrajectoryGenerator.generateTrajectory(
        //         swerve.getPose(),
        //         List.of(),
        //         targetPose,
        //         config
        // );
        // }
    }

    @Override
    public void execute() {
        // if (targetPose != null) {
        // double currentTime = timer.get();
        // Trajectory.State goal = trajectory.sample(currentTime);

        // ChassisSpeeds speeds = controller.calculate(
        //         swerve.getPose(),
        //         goal,
        //         targetPose.getRotation()
        // );

        // swerve.getSwerveDrive().drive(speeds);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        // swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return true;
        // return timer.get() >= trajectory.getTotalTimeSeconds();
    }
}
