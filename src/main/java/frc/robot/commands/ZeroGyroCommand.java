// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroGyroCommand extends CommandBase {
  private final SwerveDriveSubsystem swerveDriveSubsystem;


  public ZeroGyroCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;

    addRequirements(swerveDriveSubsystem);
  }

  // Resets the Gyro Heading
  @Override
  public void initialize() {
    swerveDriveSubsystem.zeroOutGyro();
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveDriveSubsystem.getYaw() == 0;
  }
}
