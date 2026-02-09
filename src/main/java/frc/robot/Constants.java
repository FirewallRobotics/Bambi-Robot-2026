// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import java.util.Optional;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public class DriveConstants {
    public static final double maxSpeed =
        TunerConstants.kSpeedAt12Volts.in(
            Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double maxAngularRate =
        Units.RotationsPerSecond.of(0.75)
            .in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final Distance shooterSideOffset = Units.Inches.of(6.0);

    public static final Transform2d shooterTransform =
        new Transform2d(Units.Inches.of(0.0), shooterSideOffset, new Rotation2d());

    public static final Pose3d redHubPose =
        new Pose3d(
            Units.Inches.of(468.56),
            Units.Inches.of(158.32),
            Units.Inches.of(72.0),
            new Rotation3d());
    public static final Pose3d blueHubPose =
        new Pose3d(
            Units.Inches.of(152.56),
            Units.Inches.of(158.32),
            Units.Inches.of(72.0),
            new Rotation3d());

    public static final Pose3d redFerryPoseDepot = new Pose3d(14.3, 6, 0, Rotation3d.kZero);
    public static final Pose3d redFerryPoseOutpost = new Pose3d(14.3, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseDepot = new Pose3d(2.1, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseOutpost = new Pose3d(2.1, 6, 0, Rotation3d.kZero);

    public static final Angle epsilonAngleToGoal = Units.Degrees.of(1.0);

    public static final Pose3d getHubPose() {
      Pose3d pose =
          DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
      return pose;
    }

    public static final Pose3d getFerryPose(Translation2d robotPose) {
      if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
        if (robotPose.getDistance(redFerryPoseDepot.getTranslation().toTranslation2d())
            > robotPose.getDistance(redFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return redFerryPoseOutpost;
        } else {
          return redFerryPoseDepot;
        }
      } else {
        if (robotPose.getDistance(blueFerryPoseDepot.getTranslation().toTranslation2d())
            > robotPose.getDistance(blueFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return blueFerryPoseOutpost;
        } else {
          return blueFerryPoseDepot;
        }
      }
    }

    public static final PIDController rotationController = getRotationController();

    private static final PIDController getRotationController() {
      PIDController controller = new PIDController(1.5, 0.0, 0.0);
      controller.enableContinuousInput(-Math.PI, Math.PI);
      return controller;
    }
  }

  public static class VisionSubsystemConstants {
    public static final String limelightName = "limelight-cyclops";

    public static final int ApriltagsPipeline = 0;

    public static final double[] BlueHUBCenter = {4.6, 4.04};
    public static final double[] RedHUBCenter = {11.9, 4.04};

    public static final int[] HUBTags = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};
    public static final int[] trenchTags = {1, 6, 7, 12, 17, 22, 23, 28};
    public static final int[] outpostTags = {13, 14, 29, 30};
    public static final int[] towerTags = {15, 16, 31, 32};
  }

  public static class DriverAssistanceConstants {
    public static final double minDistFeet = 9;
    public static final double maxDistFeet = 13;
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
