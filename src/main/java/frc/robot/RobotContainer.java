// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.Flywheel.ManualFlywheelCommand;
import frc.robot.commands.HorizontalConveyor.ManualIntakeCommand;
import frc.robot.commands.HorizontalConveyor.ToggleIntakeCommand;
import frc.robot.commands.VerticalConveyor.ManualConveyorCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final GenericHID primaryGenericHID = new GenericHID(OperatorConstants.kDriverControllerPort);

  private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  private final HorizontalConveyorSubsystem horizontalConveyorSubsystem = new HorizontalConveyorSubsystem();
  private final VerticalConveyorSubsystem verticalConveyorSubsystem = new VerticalConveyorSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    new JoystickButton(primaryGenericHID, JoystickConstants.kBButtonPort)
    .onTrue(
      new ToggleIntakeCommand(horizontalConveyorSubsystem)
    );

    new JoystickButton(primaryGenericHID, JoystickConstants.kLeftBumperPort)
    .onTrue(
      new ParallelCommandGroup(
        new ManualIntakeCommand(primaryGenericHID, horizontalConveyorSubsystem),
        new ManualConveyorCommand(primaryGenericHID, verticalConveyorSubsystem)
      )
    );

    new JoystickButton(primaryGenericHID, JoystickConstants.kRightBumperPort)
    .onTrue(
      new ParallelCommandGroup(
        new ManualIntakeCommand(primaryGenericHID, horizontalConveyorSubsystem),
        new ManualConveyorCommand(primaryGenericHID, verticalConveyorSubsystem)
      )
    );

    new Trigger(() -> Math.abs(primaryGenericHID.getRawAxis(JoystickConstants.kLeftYJoystickPort)) >= JoystickConstants.kDeadzone)
      .onTrue(
        new ManualFlywheelCommand(primaryGenericHID, flywheelSubsystem)
    );
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /* 
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return ;
  }
  */
}
