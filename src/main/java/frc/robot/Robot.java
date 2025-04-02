// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
//import com.thethriftybot.ThriftyNova.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private static final CANcoder encoder_FL = new CANcoder(1);
  private static final CANcoder encoder_FR = new CANcoder(2);  
  private static final CANcoder encoder_BL = new CANcoder(3);
  private static final CANcoder encoder_BR = new CANcoder(4);

  // private static final SparkMax RightCoralIntake = new SparkMax(15, MotorType.kBrushless);
  // private static final SparkMax LeftCoralIntake = new SparkMax(13, MotorType.kBrushless);

  private static final SparkMax PivotMax = new SparkMax(11, MotorType.kBrushless);
  private static RelativeEncoder RELATIVE_ENCODER = PivotMax.getEncoder();
  // private double targetPosition = RELATIVE_ENCODER.getPosition(); 

  // private static final SparkMax Coral_Intake = new SparkMax(10,MotorType.kBrushless);

  private static final double SCORE = 3.857;
  //commented
  private static final double LOAD = 10.2857;

  public static double tx = 0;
  public static double ty = 0;
  public static double ta = 0;
  public static boolean hasTarget;

  private double targetPosition = 0;

//   public static Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("");

//   public static Pose2d tagPose2d = new Pose2d(
//     tagPose.getX(),
//     tagPose.getY(),
//     new Rotation2d(Math.toRadians(tagPose.getRotation().getZ()))
// );

//   public static Transform2d offset = new Transform2d(
//     new Translation2d(-1.0, 0.0), // 1m behind tag in tag frame
//     Rotation2d.fromDegrees(180)   // Face toward the tag
// );

  // private final SparkMax elevator = new SparkMax(4,MotorType.kBrushless);

  private static final XboxController XBOX_CONTROLLER = new XboxController(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    SmartDashboard.putNumber("FL Angle: ", encoder_FL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("FR Angle: ", encoder_FR.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BL Angle: ", encoder_BL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BR Angle: ", encoder_BR.getAbsolutePosition().getValueAsDouble());

    


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("XLimelight", LimelightHelpers.getTargetPose3d_RobotSpace("").toPose2d().getX());
    SmartDashboard.putNumber("YLimelight", LimelightHelpers.getTargetPose3d_RobotSpace("").toPose2d().getY());
    SmartDashboard.putNumber("DegreesLimelight", LimelightHelpers.getTargetPose3d_RobotSpace("").toPose2d().getRotation().getDegrees());
    //elevator.set(XBOX_CONTROLLER.getLeftX());

    // tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    // ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    // ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    // hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    // if (hasTarget) {
    //   tx = 0; ty = 0; ta = 0;
    // }

//     tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("");

//   tagPose2d = new Pose2d(
//     tagPose.getX(),
//     tagPose.getY(),
//     new Rotation2d(Math.toRadians(tagPose.getRotation().getZ()))
// );

//   offset = new Transform2d(
//     new Translation2d(-1.0, 0.0), // 1m behind tag in tag frame
//     Rotation2d.fromDegrees(180)   // Face toward the tag
// );

//   tagPose2d.transformBy(offset);

  // System.out.println(tagPose2d.getX());
  // System.out.println(tagPose2d.getY());
  // System.out.println(tagPose.getRotation().getAngle());

var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);

    // PivotMax.configure(turnConfig, null, PersistMode.kNoPersistParameters);

    int PIVOT_RIGHT = XBOX_CONTROLLER.getLeftBumperButton() ? 1:0;
    int PIVOT_LEFT = XBOX_CONTROLLER.getRightBumperButton() ? 1:0;

    double pivot_speed = (PIVOT_RIGHT - PIVOT_LEFT)/10.0;


    if (pivot_speed == 0) {
      double currentPosition = RELATIVE_ENCODER.getPosition();
      double error = targetPosition - currentPosition;
      PivotMax.set(error*.05);
    } else {
      PivotMax.set(pivot_speed);
    }

    if (XBOX_CONTROLLER.getRightBumperButtonReleased() || XBOX_CONTROLLER.getLeftBumperButtonReleased()) {
      targetPosition = RELATIVE_ENCODER.getPosition();
    }

    /*if (XBOX_CONTROLLER.getRightBumperButton() ) {
      double currentPosition = RELATIVE_ENCODER.getPosition();
      double error = LOAD - currentPosition;
      PivotMax.set(error*.1);
    } if (XBOX_CONTROLLER.getLeftBumperButton()) {
      double currentPosition = RELATIVE_ENCODER.getPosition();
      double error = - currentPosition;
      PivotMax.set(error*.1);
    }


    if (XBOX_CONTROLLER.getRightBumperButton()) {
      System.out.println(Coral_Intake.getAbsoluteEncoder().getPosition());
      coralIntakePID.calculate(Coral_Intake.getAbsoluteEncoder().getPosition(), 0.5);
    }
    */
    /* 
    System.out.println(PIVOT_LEFT);
    System.out.println(PIVOT_RIGHT);
    */
    double intake = XBOX_CONTROLLER.getAButton() ? 1:0;
    double outtake = XBOX_CONTROLLER.getYButton() ? 1:0;

    double algaeIntake = XBOX_CONTROLLER.getXButton() ? 1 : 0;
    double algaeOutake = XBOX_CONTROLLER.getBButton() ? 1 : 0;

    // if (XBOX_CONTROLLER.getRightBumperButtonReleased() || XBOX_CONTROLLER.getLeftBumperButtonReleased())
    // {
    // targetPosition = RELATIVE_ENCODER.getPosition();
    // System.out.println(targetPosition);
    // }
    // System.out.println(pivot_speed);

    // if (XBOX_CONTROLLER.getRightBumperButtonReleased()) {
    //   PivotMax.set(-.05);
    // }

    //Coral_Intake.set(intake-outtake);
    //LeftCoralIntake.set(algaeIntake-algaeOutake);
    // RightCoralIntake.set(-(algaeIntake-algaeOutake));

    SmartDashboard.putNumber("FL Angle: ", encoder_FL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("FR Angle: ", encoder_FR.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BL Angle: ", encoder_BL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BR Angle: ", encoder_BR.getAbsolutePosition().getValueAsDouble());

    
  }

  @Override
  public void testInit() {



    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
