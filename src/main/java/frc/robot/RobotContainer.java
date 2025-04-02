package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.MoveToPoseCommand;
import frc.robot.commands.PivotToAngleCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

// private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(14);

  private final LimelightSubsystem limelight = new LimelightSubsystem();

  private boolean isFlipped = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

private final CommandXboxController m_operatorController = 
    new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand(
      "Score",
      Commands.sequence(
        new MoveElevatorToPosition(elevatorSubsystem, 200),
        new MoveElevatorToPosition(elevatorSubsystem, 0)
      )
    );
    
    configureBindings();
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleKeyboard);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> m_driverController.getLeftY(), () -> m_driverController.getLeftX())
  .withControllerRotationAxis(m_driverController::getRightX).deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> (m_driverController.getRightX()*-1), m_driverController::getRightY).headingWhile(true);

  Command driveFieldDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

//Similar to code overhead, just used for autonomous driving 

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));


  Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  // public Command generateAndRunPath() {
  //   List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
  //       new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
  //       new Pose2d(0.1, 0, Rotation2d.fromDegrees(0))
  //     );

  //     PathConstraints constraints = new PathConstraints(0.25, 1.0, 1 * Math.PI, 2 * Math.PI); // The constraints for this path.
  //     // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

  //     // Create the path using the waypoints created above
  //     PathPlannerPath path = new PathPlannerPath(
  //             waypoints,
  //             constraints,
  //             null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
  //             new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  //     );

  //     // Prevent the path from being flipped if the coordinates are already correct
  //     path.preventFlipping = true;

      
  //     drivebase.getSwerveDrive().addVisionMeasurement(new Pose2d(0.0, 0.0, new Rotation2d(0)), Timer.getTimestamp());

  //     return AutoBuilder.followPath(path);
    
  // }
  private void configureBindings() {
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    new JoystickButton(m_driverController.getHID(), 1).onTrue(
        new MoveToPoseCommand(drivebase)
    );

    

    new JoystickButton(m_operatorController.getHID(), 1)
            .onTrue(new MoveElevatorToPosition(elevatorSubsystem, 0));

        // Example: Move to 0 (home) when Button 2 is pressed
        new JoystickButton(m_operatorController.getHID(), 2)
            .onTrue(new MoveElevatorToPosition(elevatorSubsystem, 150));

        // new JoystickButton(m_operatorController.getHID(), 5)
        //     .onTrue(new PivotToAngleCommand(pivotSubsystem.getPivotMotor(), 0));

        // new JoystickButton(m_operatorController.getHID(), 6)
        //     .onTrue(new PivotToAngleCommand(pivotSubsystem.getPivotMotor(), 180));

//         new JoystickButton(m_driverController.getHID(), XboxController.Button.kA.value)
//     .onTrue(new InstantCommand(() -> {
//         if (limelight.hasTarget()) {
//     Pose2d tagPose = limelight.getBotPose2d();

//     // Move 1 meter behind the tag (in tag’s coordinate frame)
//     Pose2d targetPose = tagPose.transformBy(
//         new Transform2d(new Translation2d(-1.0, 0.0), new Rotation2d())
//     );

//     new DriveToPose(
//         drivebase,
//         targetPose.getY(),           // forward
//         targetPose.getX(),           // right
//         targetPose.getRotation()     // heading
//     ).schedule();
// }
//     }));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New Auto");
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return drivebase;
}
}
