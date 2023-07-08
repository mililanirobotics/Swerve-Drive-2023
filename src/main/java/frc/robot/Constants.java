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
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    // Drive Port Constants
    public static final int kLeftFrontWheelPort = 1;
    public static final int kLeftFrontRotationPort = 2;
    public static final int kLeftFrontAbsoluteEncoderPort = 0;

    public static final int kRightFrontWheelPort = 3;
    public static final int kRightFrontRotationPort = 4;
    public static final int kRightFrontAbsoluteEncoderPort = 1;

    public static final int kLeftBackWheelPort = 5;
    public static final int kLeftBackRotationPort = 6;
    public static final int kLeftBackAbsoluteEncoderPort = 2;

    public static final int kRightBackWheelPort = 7;
    public static final int kRightBackRotationPort = 8;
    public static final int kRightBackAbsoluteEncoderPort = 3;

    public static final int kAbsoluteEncoderOffset = .20;
    

    // Reverse Booleans
    public static final boolean kLeftFrontReversed = false;
    public static final boolean kRightFrontReversed = true;
    public static final boolean kLeftBackReversed = false;
    public static final boolean kRightBackReversed = true;
  }
}
