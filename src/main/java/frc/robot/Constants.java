// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

    public static final double kRotationGearRatio = 1 / (150 / 7.0);
    public static final double kDriveGearRatio = 1/ 8.14;
    public static final double kWheelDiameter = Units.inchesToMeters(4);

    // Drive Port Constants
    public static final int kLeftFrontWheelPort = 13;
    public static final int kLeftFrontRotationPort = 14;
    public static final int kLeftFrontCANCoderPort = 3;

    public static final int kRightFrontWheelPort = 5;
    public static final int kRightFrontRotationPort = 6;
    public static final int kRightFrontCANCoderPort = 1;

    public static final int kLeftBackWheelPort = 15;
    public static final int kLeftBackRotationPort = 16;
    public static final int kLeftBackCANCoderPort = 2;

    public static final int kRightBackWheelPort = 3;
    public static final int kRightBackRotationPort = 4;
    public static final int kRightBackCANCoderPort = 4;

    public static final double kLeftFrontCANCoderOffset = 4.9;
    public static final double kRightFrontCANCoderOffset = 3.3;
    public static final double kLeftBackCANCoderOffset = 4.25;
    public static final double kRightBackCANCoderOffset = 1.59;
    

    // Reverse Booleans
    public static final boolean kLeftFrontDriveReversed = false;
    public static final boolean kRightFrontDriveReversed = true;
    public static final boolean kLeftBackDriveReversed = false;
    public static final boolean kRightBackDriveReversed = true;

    public static final boolean kLeftFrontRotationReversed = true;
    public static final boolean kRightFrontRotationReversed = true;
    public static final boolean kLeftBackRotationReversed = true;
    public static final boolean kRightBackRotationReversed = true;

    public static final boolean kLeftFrontCANCoderReversed = false;
    public static final boolean kRightFrontCANCoderReversed = false;
    public static final boolean kLeftBackCANCoderReversed = false;
    public static final boolean kRightBackCANCoderReversed = false;

    // Conversion Units
    public static final double kRotationToMeters = kDriveGearRatio * Math.PI * kWheelDiameter;
    public static final double kRotationToRadians = kRotationGearRatio * 2 * Math.PI;

    // Measurement Units
    public static final double kMetersPerSecond = kRotationToMeters / 60.0 ;
    public static final double kRadiansPerSecond = kRotationToRadians / 60.0;

    // PID Constants
    public static final double kTurningP = 0.6;
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
      leftFrontLocation, 
      rightFrontLocation,
      leftBackLocation,
      rightBackLocation
    );
  }

  public static class DriveConstants {

    // Drive Speed Constants
    public static final double kDriveMaxMetersPerSecond = 3;
    public static final double kRotationMaxRadiansPerSecond = 3;

    public static final double kTeleDriveMaxAcceleration = kDriveMaxMetersPerSecond;
    public static final double kTeleRotationMaxAngularAcceleration = kRotationMaxRadiansPerSecond;
  }

  public static class AutoConstants {
    public static final double kAutoDriveMaxMetersPerSecond = 4;
    public static final double kAutoDriveMaxAcceleration = kAutoDriveMaxMetersPerSecond;
    public static final double kAutoDriveMaxRadiansPerSecond = 1.5;
    public static final double kAutoDriveMaxAngularAcceleration = kAutoDriveMaxRadiansPerSecond;

    //PID Constants
    public static final double kPXController = 0.5;
    public static final double kIXController = 0;
    public static final double kDXController = 0.006;

    public static final double kPYController = 0.5;
    public static final double kIYController = 0;
    public static final double kDYController = 0.006;

    public static final double kPThetaController = 0.45;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = -0.15;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(
        kAutoDriveMaxRadiansPerSecond, 
        kAutoDriveMaxAngularAcceleration
      );

    //================================
    // Trajectory
    //================================
    
    private static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      kAutoDriveMaxMetersPerSecond,
      kAutoDriveMaxAcceleration
    )
    .setKinematics(SwerveModuleConstants.kinematics);

    public static final Trajectory testDrive = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(0, 1),
          new Translation2d(0.5, 1),
          new Translation2d(0.5, 0)
        ),
        new Pose2d(0, 0, new Rotation2d(0)),
        trajectoryConfig
      );
    
    public static final Trajectory figureEightPath = 
      TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        //first point (-0.25, 0.5)
        new Translation2d(-0.25, 0.5),
        //second point (-0.5, 0)
        new Translation2d(-0.5, 0),
        //third point (-0.25, -0.5)
        new Translation2d(-0.25, -0.5),
        //fourth point (0, 0)
        new Translation2d(0, 0),
        //fifth point (0.25, 0.5)
        new Translation2d(0.25, 0.5),
        //sixth point (0.5, 0)
        new Translation2d(0.5, 0),
        //seventh point (0.25, -0.5)
        new Translation2d(0.25, -0.5)
      ),
      new Pose2d(0, 0, Rotation2d.fromRadians(Math.PI)),
      trajectoryConfig      
    );
  }

  public static class JoystickConstants {
    public static final int kPrimaryGamepadPort = 0;

    // Gamepad Axis Ports
    public static final int kleftXJoystickPort = 0;
    public static final int kLeftYJoystickPort = 1;
    public static final int kRightXJoystickPort = 4;
    public static final int kRightYJoystickPort = 5; 

    // Gamepad Button Ports
    public final static int kAButtonPort = 1;
    public final static int kBButtonPort = 2;
    
    // Deadband constant to correct minor joystick inputs
    public static final double kDeadband = 0.07;
  }

}
