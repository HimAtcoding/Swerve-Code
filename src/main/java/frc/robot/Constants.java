// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce ity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = .05;
  }

  public static final double maxSpeed = Units.feetToMeters(16.5);

  //"absoluteEncoderOffset": 0.81298828125, 
  

  // For Alignment PID
  public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(5);
  public static final TrapezoidProfile.Constraints kVelocityConstraints = new TrapezoidProfile.Constraints(
      maxSpeed, 3);

  public static final TrapezoidProfile.Constraints kThetaConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularRate.in(RotationsPerSecond), 3);
  public static final ProfiledPIDController kPoseVelocityXController = new ProfiledPIDController(5.5, 0, 0,
      kVelocityConstraints);
  public static final ProfiledPIDController kPoseVelocityYController = new ProfiledPIDController(5.5, 0, 0,
      kVelocityConstraints);
  public static final ProfiledPIDController kPoseThetaController = new ProfiledPIDController(10, 0, 0,
      kThetaConstraints);
      
  public static final Translation2d leftReefOffset = new Translation2d(0.422, -0.1651); // robot bumper offset to be flush and then the left
    public static final Translation2d rightReefOffset = new Translation2d(0.422, 0.1651);// robot bumper offset to be flush and then the right

  // Elevator PID Constants (TUNE THESE)
  public static final double ELEVATOR_kP = 0.1;
  public static final double ELEVATOR_kI = 0.0;
  public static final double ELEVATOR_kD = 0.01;

  // Elevator Setpoints
  public static final double ELEVATOR_LOW = 0.0;  // Lowest position
  public static final double ELEVATOR_HIGH = 100.0;  // Highest position
  public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(.5, .5, 9999999);

  public static int[] reefIds = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
  public static Map<Integer, Pose2d> getReefPoseMap() {
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);  
    Map<Integer, Pose2d> poseMap = new HashMap<>();
    for (int tagId : reefIds) {
      Optional<Pose3d> optionalPose = fieldLayout.getTagPose(tagId);

      if (optionalPose.isPresent()) {
        Pose2d pose = optionalPose.get().toPose2d();
        poseMap.put(tagId, pose);
      }
    }
    return poseMap;
  }
  public static Map<Integer, Pose2d> reefPoseMap = getReefPoseMap();

  public enum ScoringPosition {
      LEFT,
      RIGHT
    }
}
