// package frc.robot;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;


// public class DriveToVisionTarget extends Command {
//     private final SwerveSubsystem swerve;
//     private final LimelightSubsystem limelight;
//     private final PIDController turnController = new PIDController(0.02, 0, 0);

//     public DriveToVisionTarget(SwerveSubsystem swerve, LimelightSubsystem limelight) {
//         this.swerve = swerve;
//         this.limelight = limelight;
//         addRequirements(swerve);
//     }

//     public void execute() {
//         if (!limelight.hasTarget()) {
//             swerve.stopModules();
//             return;
//         }

//         double tx = limelight.getTX(); // Horizontal offset from crosshair to target
//         double turnSpeed = turnController.calculate(tx, 0); // Want tx = 0

//         swerve.drive(new Translation2d(0, 0), turnSpeed, true, false);
//     }

//     public boolean isFinished() {
//         return Math.abs(limelight.getTX()) < 1.0;
//     }

//     public void end(boolean interrupted) {
//         swerve.stopModules();
//     }
// }
