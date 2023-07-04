// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class OperatorConstants {
    public final static int kDriverControllerPort = 0;
  }

  public final static class FlywheelConstants {
    public final static int kLeftFlywheel = 16;
    public final static int kRightFlywheel = 17;

    public final static boolean kLeftFlywheelReverse = false;
    public final static boolean kRightFlywheelReverse = true;
  }

  public final static class HorizontalConveyorConstants {
    public final static int kHorizontalConveyor = 18;
    public final static int kHorizontalIntake = 30;

    public final static boolean kHorizontalConveyorReverse = true;
    public final static boolean kHorizontalIntakeReverse = false;

    public final static int kLeftSolenoidForwardChannel = 0;
    public final static int kLeftSolenoidReverseChannel = 1;

    public final static int kRightSolenoidForwardChannel = 2;
    public final static int kRightSolenoidReverseChannel = 3;
  }

  public final static class VerticalConveyorConstants {
    public final static int kLeftVerticalConveyor = 20;
    public final static int kRightVerticalConveyor = 19;

    public final static boolean kLeftVerticalConveyorReverse = true;
    public final static boolean kRightVerticalConveyorReverse = false;
  }

  public final static class RobotConstants {
    public final static int kCountsPerRev = 42;
  }
  
  public static final class JoystickConstants {
    //joystick port for the gamepad
    public final static int kPrimaryLeftStickPort = 0;
    public final static int kPrimaryRightStickPort = 1;
    public final static int kSecondaryPort = 2;

    //Gamepad ports
    public final static int kAButtonPort = 1;
    public final static int kBButtonPort = 2;
    public final static int kXButtonPort = 3;
    public final static int kYButtonPort = 4;
    public final static int kLeftBumperPort = 5;
    public final static int kRightBumperPort = 6;
    public final static int kBackButtonPort = 7;
    public final static int kStartButtonPort = 8;

    //Gamepad axis ports
    public final static int kLeftYJoystickPort = 1;
    public final static int kLeftTriggerPort = 2;
    public final static int kRightTriggerPort = 3;
    public final static int kRightYJoystickPort = 5;

    //Dpad values
    public final static int kDpadUp = 0;
    public final static int kDpadDown = 180;

    //Attack3 button ports
    public final static int kAttackTriggerPort = 1;
    public final static int kAttackButtonTwo = 2;

    //Attack3
    public final static int kAttackYAxisPort = 1;

    //joystick deadzones
    public final static double kDeadzone = 0.1;
}
}
