package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;

  // pipeline layout:
  // 0 - april tags
  // 1 - Reef Target
  // 2 - Coral Station Target

  private static int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  private static int[] coralTags = {1, 2, 12, 13};
  private static int[] processorTags = {3, 16};
  private static int[] bargeTags = {4, 5, 14, 15};
  boolean doRejectUpdate;

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ReefDistance", VisionSubsystem.DistanceToReef());
    SmartDashboard.putNumber("CoralStationDistance", VisionSubsystem.DistanceToCoralStation());
    SmartDashboard.putNumber("ProcessorDistance", VisionSubsystem.DistanceToProcessor());

    /*
    LimelightHelpers.SetRobotOrientation(
        name, RobotContainer.drivebase.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (mt2 != null) {
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      } else {
        doRejectUpdate = false;
      }
      if (!doRejectUpdate) {
        RobotContainer.drivebase.addVisionReading(mt2.pose, mt2.timestampSeconds);
      }
    }
    */
  }

  public static int[] getTags() {
    LimelightHelpers.setPipelineIndex(name, 0);
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }
    int[] temp = new int[results.targets_Fiducials.length];
    for (int i = 0; i < results.targets_Fiducials.length; i++) {
      temp[i] = (int) results.targets_Fiducials[i].fiducialID;
    }
    return temp;
  }

  public static boolean CanSeeTag(int tag) {
    LimelightHelpers.setPipelineIndex(name, 0);
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }
    for (LimelightTarget_Fiducial SeenTag : results.targets_Fiducials) {
      if (SeenTag.fiducialID == tag) {
        return true;
      }
    }
    return false;
  }

  public static Pose3d getRobotPoseInFieldSpace() {
    if (!Robot.isSimulation()) {
      LimelightHelpers.setPipelineIndex(name, 0);
      LimelightResults results = LimelightHelpers.getLatestResults(name);
      // if the limelights intel is good look for reef tag
      while (!results.valid) {
        results = LimelightHelpers.getLatestResults(name);
      }
      LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
      return tag.getRobotPose_FieldSpace();
    } else {
      return null;
    }
  }

  public static boolean CanSeeAlgae() {
    LimelightHelpers.setPipelineIndex(
        frc.robot.Constants.VisionSubsystemConstants.limelightName, 3);
    if (LimelightHelpers.getTargetColor(name)[0] != -1) {
      return true;
    }
    return false;
  }

  public static double[] getReefLocation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int reeftag : reefTags) {
        if (tag.fiducialID == reeftag) {

          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a reef tag
        break;
      }
    }
    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public static double[] getCoralStationLocation() {

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int coraltag : coralTags) {
        if (tag.fiducialID == coraltag) {

          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a reef tag
        break;
      }
    }
    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public static double[] getProcessorLocation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int processorTag : processorTags) {
        if (tag.fiducialID == processorTag) {

          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a reef tag
        break;
      }
    }
    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public static Pose3d getReefLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int reeftag : reefTags) {
        if (tag.fiducialID == reeftag) {

          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a reef tag
        break;
      }
    }
    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  public static Pose3d getCoralStationLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int coraltag : coralTags) {
        if (tag.fiducialID == coraltag) {

          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a reef tag
        break;
      }
    }
    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  public static Pose3d getProcessorLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int processorTag : processorTags) {
        if (tag.fiducialID == processorTag) {

          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a reef tag
        break;
      }
    }

    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  public static double DistanceToReef() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(
        frc.robot.Constants.VisionSubsystemConstants.limelightName, 0);

    // get the results
    RawFiducial[] fiducials =
        LimelightHelpers.getRawFiducials(
            frc.robot.Constants.VisionSubsystemConstants.limelightName);

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all reef tags to find if this is a reef tag
      for (int i = 0; i < reefTags.length; i++) {

        // if it is then make it the new shortest
        if (id == reefTags[i]) {
          shortest = distToRobot;
        }
      }
    }

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public static double DistanceToCoralStation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all coral Station tags to find if this is a coral Station tag
      for (int i = 0; i < coralTags.length; i++) {

        // if it is then make it the new shortest
        if (id == coralTags[i]) {
          shortest = distToRobot;
        }
      }
    }

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public static double DistanceToProcessor() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all processor tags to find if this is a processor tag
      for (int i = 0; i < processorTags.length; i++) {

        // if it is then make it the new shortest
        if (id == processorTags[i]) {
          shortest = distToRobot;
        }
      }
    }

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public static double DistanceToBarge() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all barge tags to find if this is a barge tag
      for (int i = 0; i < bargeTags.length; i++) {

        // if it is then make it the new shortest
        if (id == bargeTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    // set pipeline to the what it was before
    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }
}
