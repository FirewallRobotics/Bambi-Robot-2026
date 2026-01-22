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
  public static class VisionSubsystemConstants {
    public static final String limelightName = "limelight-cyclops";

    public static final int ApriltagsPipeline = 0;

    public static final int[] HUBTags = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};
    public static final int[] trenchTags = {1, 6, 7, 12, 17, 22, 23, 28};
    public static final int[] outpostTags = {13, 14, 29, 30};
    public static final int[] towerTags = {15, 16, 31, 32};
  }

  public static class ArmConstants {
    
    public static final int IntakeMotorID = 0;
    public static final int ArmMotorID = 1;

    public static final double kSVolts = 0.11356;
    public static final double kGVolts = 0.29175;
    public static final double kVVoltSecondPerRad = 1.5928;
    public static final double kAVoltSecondSquaredPerRad = 0.030171;
  }
}
