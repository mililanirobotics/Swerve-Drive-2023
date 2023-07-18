// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveModuleConstants {

    public static final int kDriverControllerPort = 0;

    // Drive Port Constants
    public static final int kLeftFrontWheelPort = 20;
    public static final int kLeftFrontRotationPort = 10;
    public static final int kLeftFrontCANCoderPort = 10;

    public static final int kRightFrontWheelPort = 3;
    public static final int kRightFrontRotationPort = 4;
    public static final int kRightFrontCANCoderPort = 1;

    public static final int kLeftBackWheelPort = 5;
    public static final int kLeftBackRotationPort = 6;
    public static final int kLeftBackCANCoderPort = 2;

    public static final int kRightBackWheelPort = 7;
    public static final int kRightBackRotationPort = 8;
    public static final int kRightBackCANCoderPort = 3;

    public static final double kAbsoluteEncoderOffset = .20;
    

    // Reverse Booleans
    public static final boolean kLeftFrontReversed = false;
    public static final boolean kRightFrontReversed = true;
    public static final boolean kLeftBackReversed = false;
    public static final boolean kRightBackReversed = true;

    // Conversion Units
    public static final double kRotationToMeters = 0;
    public static final double kRotationToRadians = 0;

    // Measurement Units
    public static final double kMetersPerSecond = 0;
    public static final double kRadiansPerSecond = 0;

    // PID Constants

    public static final double kTurningP = 0;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;

    // The distances between each Module from the center of the robot (Meters)
    public static final double kModuleDistance = 0.312;

    // 2d translation coordinates relative to center 

    public static final double kLeftFront2dX = kModuleDistance;
    public static final double kLeftFront2dY = kModuleDistance;

    public static final double kRightFront2dX = kModuleDistance;
    public static final double kRightFront2dY = -kModuleDistance;

    public static final double kLeftBack2dX = -kModuleDistance;
    public static final double kLeftBack2dY = kModuleDistance;

    public static final double kRightBack2dX = -kModuleDistance;
    public static final double kRightBack2dY = -kModuleDistance;

    public static final Translation2d leftFrontLocation = new Translation2d(
      SwerveModuleConstants.kLeftFront2dX, 
      SwerveModuleConstants.kLeftFront2dY
    );

    public static final Translation2d rightFrontLocation = new Translation2d(
      SwerveModuleConstants.kRightFront2dX,
      SwerveModuleConstants.kRightFront2dY
    );

    public static final Translation2d leftBackLocation = new Translation2d(
      SwerveModuleConstants.kLeftBack2dX,
      SwerveModuleConstants.kLeftBack2dY
    );

    public static final Translation2d rightBackLocation = new Translation2d(
      SwerveModuleConstants.kRightBack2dX,
      SwerveModuleConstants.kRightBack2dY
    );

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation
    );
  }

  public static class DriveConstants {

    // Drive Speed Constants
    public static final double kDriveMaxMetersPerSecond = 1;
    public static final double kRotationMaxRadiansPerSecond = 1.5;

    public static final double kTeleDriveMaxMetersPerSecond = kDriveMaxMetersPerSecond / 4;
    public static final double kTeleRotationMaxRadiansPerSecond = kRotationMaxRadiansPerSecond / 4;

  }

  public static class JoystickConstants {
    public static final int kPrimaryGamepadPort = 0;

    // Gamepad Axis Ports
    public static final int kleftXJoystickPort = 0;
    public static final int kLeftYJoystickPort = 1;
    public static final int kRightXJoystickPort = 2;
    public static final int kRightYJoystickPort = 3; 

    // Gamepad Button Ports
    public final static int kAButtonPort = 1;
    public final static int kBButtonPort = 2;
    
    // Deadband constant to correct minor joystick inputs
    public static final double kDeadband = 0.05;
  }

}
