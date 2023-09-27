// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.JoystickConstants;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// Subsystem Imports
import frc.robot.subsystems.SwerveDriveSubsystem;

// Command Imports
import frc.robot.commands.SwerveControlCommand;
import frc.robot.commands.ZeroGyroCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

  private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.kAutoDriveMaxMetersPerSecond,
    AutoConstants.kAutoDriveMaxAcceleration
  )
  .setKinematics(SwerveModuleConstants.kinematics);

  private final GenericHID primaryGamepad = new GenericHID(JoystickConstants.kPrimaryGamepadPort); 

  private SendableChooser<CommandBase> autoCommand = new SendableChooser<CommandBase>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    swerveDriveSubsystem.setDefaultCommand(new SwerveControlCommand(
        swerveDriveSubsystem, 
        () -> primaryGamepad.getRawAxis(JoystickConstants.kLeftYJoystickPort),
        () -> primaryGamepad.getRawAxis(JoystickConstants.kleftXJoystickPort),
        () -> primaryGamepad.getRawAxis(JoystickConstants.kRightXJoystickPort),
        primaryGamepad
      )
    );

    // autoCommand.addOption("Test Figure Eight", runTrajectory(
    //     TrajectoryGenerator.generateTrajectory(
    //       new Pose2d(0, 0, swerveDriveSubsystem.getRotation2dContinuous()),
    //       AutoConstants.figureEightPath, 
    //       new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
    //       trajectoryConfig
    //     )
    //   )
    // );

    autoCommand.addOption("Test Straight Line", runTrajectory(
      PathPlanner.generatePath(
        new PathConstraints(AutoConstants.kAutoDriveMaxMetersPerSecond,
          AutoConstants.kAutoDriveMaxAcceleration), 
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), 
          new PathPoint(new Translation2d(1, 0), Rotation2d.fromDegrees(180)),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0))
        )
      )
    );  

    PathPlannerTrajectory straightTrajectory = PathPlanner.generatePath(
        new PathConstraints(AutoConstants.kAutoDriveMaxMetersPerSecond,
        AutoConstants.kAutoDriveMaxAcceleration), 
        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), 
        new PathPoint(new Translation2d(0.5, 0), Rotation2d.fromDegrees(90)),
        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0))
      );

    PathPlannerTrajectory john = PathPlanner.loadPath("God's Plan", new PathConstraints(
        AutoConstants.kAutoDriveMaxMetersPerSecond, AutoConstants.kAutoDriveMaxAcceleration
      )
    );

    autoCommand.addOption("John", runTrajectory(john));

    SmartDashboard.putData("auto", autoCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(primaryGamepad, JoystickConstants.kAButtonPort).onTrue(
      new ZeroGyroCommand(swerveDriveSubsystem)
    ); 
  }

  public CommandBase runTrajectory(PathPlannerTrajectory generatedTrajectory) {
    PIDController xController = new PIDController(
      AutoConstants.kPXController, 
      AutoConstants.kIXController,
      AutoConstants.kDXController
    );

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController
    );

    PIDController thetaController = new PIDController(
      AutoConstants.kPThetaController,
      AutoConstants.kIThetaController,
      AutoConstants.kDThetaController
    );
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveTrajectory = new PPSwerveControllerCommand(
      generatedTrajectory,
      swerveDriveSubsystem::getPose,
      SwerveModuleConstants.kinematics,
      xController,
      yController,
      thetaController,
      swerveDriveSubsystem::setModuleStates,
      swerveDriveSubsystem
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveDriveSubsystem.resetOdometry(generatedTrajectory.getInitialPose())),
      new WaitCommand(1),
      swerveTrajectory,
      new InstantCommand(() -> swerveDriveSubsystem.shutdown())
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoCommand.getSelected();
  }
}