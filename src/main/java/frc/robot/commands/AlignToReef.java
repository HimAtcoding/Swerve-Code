// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScoringPosition;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;

public class AlignToReef extends Command {
  private final SwerveSubsystem drivetrain;
  private final ScoringPosition scoringPosition;
  private Pose2d targetPose;
  private final int targetReefId;

  private boolean slowMode = false;

  private final ProfiledPIDController pidX = Constants.kPoseVelocityXController;
  private final ProfiledPIDController pidY = Constants.kPoseVelocityYController;
  private final ProfiledPIDController pidTheta = Constants.kPoseThetaController;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * Creates a new AlignToReef command with a slow mode option.
   *
   * @param drivetrain The drivetrain subsystem
   * @param scoringPosition The scoring position (LEFT or RIGHT)
   * @param targetReefId The specific reef ID to target. Use -1 to auto-select
   * @param slowMode Whether to enable slow mode (reduced speed when elevator is extended)
   */
  public AlignToReef(
      SwerveSubsystem drivetrain,
      ScoringPosition scoringPosition,
      int targetReefId,
      boolean slowMode) {
    this.drivetrain = drivetrain;
    this.scoringPosition = scoringPosition;
    this.targetReefId = targetReefId;
    this.slowMode = slowMode;
    addRequirements(drivetrain);
  }

  /**
   * Creates a new AlignToReef command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param scoringPosition The scoring position (LEFT or RIGHT)
   * @param targetReefId The specific reef ID to target. Use -1 to auto-select closest.
   */
  public AlignToReef(
      SwerveSubsystem drivetrain, ScoringPosition scoringPosition, int targetReefId) {
    this(drivetrain, scoringPosition, targetReefId, false);
  }

  /**
   * Creates a new AlignToReef command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param scoringPosition The scoring position (LEFT or RIGHT)
   * @param slowMode Whether to enable slow mode (reduced speed when elevator is extended)
   */
  public AlignToReef(
      SwerveSubsystem drivetrain, ScoringPosition scoringPosition, boolean slowMode) {
    this(
        drivetrain, scoringPosition, -1, slowMode); // Default to -1 if no specific reef ID is given
  }

  /**
   * Creates a new AlignToReef command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param scoringPosition The scoring position (LEFT or RIGHT)
   */
  public AlignToReef(SwerveSubsystem drivetrain, ScoringPosition scoringPosition) {
    this(drivetrain, scoringPosition, -1); // Default to -1 if no specific reef ID is given
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = drivetrain.getPose();
    resetPID(robotPose);

    Map<Integer, Pose2d> reefPoseMap = Constants.reefPoseMap;

    Pose2d selectedPose = null;
    int selectedTagId = -1;

    if (targetReefId >= 0) {
      for (Map.Entry<Integer, Pose2d> entry : reefPoseMap.entrySet()) {
        int tagId = entry.getKey();
        if (tagId == targetReefId) {
          selectedPose = entry.getValue();
          selectedTagId = tagId;
          break;
        }
      }
      if (selectedPose != null) SmartDashboard.putString("Target Selection Mode", "Specific ID");
      else SmartDashboard.putString("Target Selection Mode", "ID Not Found, Defaulting to Closest");
    }

    if (selectedPose == null) {
      double minDistance = Double.MAX_VALUE;
      for (Map.Entry<Integer, Pose2d> entry : reefPoseMap.entrySet()) {
        int tagId = entry.getKey();
        Pose2d tagPose = entry.getValue();
        double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          selectedTagId = tagId;
          selectedPose = tagPose;
        }
      }
      SmartDashboard.putString("Target Selection Mode", "Closest Tag");
    }

    SmartDashboard.putNumber("Selected Reef Tag", selectedTagId);

    if (selectedPose == null) {
      System.err.println("No valid reef tag pose available!");
      return;
    }

    if (scoringPosition == ScoringPosition.LEFT) {
      targetPose =
          selectedPose.transformBy(
              new Transform2d(Constants.leftReefOffset, new Rotation2d()));
    } else {
      targetPose =
          selectedPose.transformBy(
              new Transform2d(Constants.rightReefOffset, new Rotation2d()));
    }

    targetPose =
        targetPose.transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(-Math.PI)));

    SmartDashboard.putNumberArray(
        "Target Robot Reef Pose",
        new double[] {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});

    pidX.setTolerance(0.02);
    pidY.setTolerance(0.02);
    pidTheta.setTolerance(Units.degreesToRadians(0.1));

    if (slowMode) {
      TrapezoidProfile.Constraints velocityConstraints = new TrapezoidProfile.Constraints(1.5, 1.5);
      TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(4, 2);
      pidX.setConstraints(velocityConstraints);
      pidY.setConstraints(velocityConstraints);
      pidTheta.setConstraints(thetaConstraints);
    } else {
      pidX.setConstraints(Constants.kVelocityConstraints);
      pidY.setConstraints(Constants.kVelocityConstraints);
      pidTheta.setConstraints(Constants.kThetaConstraints);
    }
    pidTheta.enableContinuousInput(-Math.PI, Math.PI);

    pidX.setGoal(targetPose.getX());
    pidY.setGoal(targetPose.getY());
    pidTheta.setGoal(targetPose.getRotation().getRadians());
  }

  public boolean atGoal() {
    boolean positionReached = pidX.atGoal() && pidY.atGoal();
    boolean angleReached = pidTheta.atGoal();

    return positionReached && angleReached;
  }

  private void resetPID(Pose2d robotPose) {
    pidX.reset(robotPose.getX());
    pidY.reset(robotPose.getY());
    pidTheta.reset(robotPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    double vx = pidX.calculate(currentPose.getX());
    double vy = pidY.calculate(currentPose.getY());
    double omega = pidTheta.calculate(currentPose.getRotation().getRadians());

    SmartDashboard.putNumber("PID vx", vx);
    SmartDashboard.putNumber("PID vy", vy);
    SmartDashboard.putNumber("PID omega", omega);

    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    //   vx *= -1;
    //   vy *= -1;
    // }

    final double vxf = vx;
    final double vyf = vy;
    final double omegaf = omega;
    drivetrain.drive(new Translation2d(vxf, vyf), omegaf, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atGoal();
  }
}