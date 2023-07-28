// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroGyroCommand extends CommandBase {
  private final SwerveDriveSubsystem m_SwerveDriveSubsystem;


  public ZeroGyroCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
    m_SwerveDriveSubsystem = swerveDriveSubsystem;

    addRequirements(m_SwerveDriveSubsystem);
  }

  // Resets the Gyro Heading
  @Override
  public void initialize() {
    m_SwerveDriveSubsystem.zeroOutGyro();
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_SwerveDriveSubsystem.getYaw() == 0;
  }
}
