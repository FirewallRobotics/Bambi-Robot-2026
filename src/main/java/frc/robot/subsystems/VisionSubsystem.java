package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;

/** This subsystem handles the high level code for specificly
 * finding and predicting the position of the robot and apriltags associated with objects in our visual field */
public class VisionSubsystem extends SubsystemBase {

  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;

  // pipeline layout:
  // 0 - april tags

  private static int[] HUBTags = {2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27};
  private static int[] trenchTags = {1,6,7,12,17,22,23,28};
  private static int[] outpostTags = {13,14,29,30};
  private static int[] towerTags = {15,16,31,32};
  boolean doRejectUpdate;

  @Override
  public void periodic() {
    SmartDashboard.putNumber("HUBDistance", VisionSubsystem.DistanceToHUB());
    SmartDashboard.putNumber("TrenchDistance", VisionSubsystem.DistanceToTrench());
    SmartDashboard.putNumber("OutpostDistance", VisionSubsystem.DistanceToOutpost());
    SmartDashboard.putNumber("TowerDistance", VisionSubsystem.DistanceToTower());

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

  /** Outputs all the tags that we can see */
  public static int[] getTags() {

    // change the pipelime to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the latest results from the limelight
    LimelightResults results = LimelightHelpers.getLatestResults(name);

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // create a list of tag numbers that has as many elements as apriltags we can see
    int[] temp = new int[results.targets_Fiducials.length];

    // loop through all the tags we can see and add them to the list
    for (int i = 0; i < results.targets_Fiducials.length; i++) {

      // get the ID of tag i and put it in the list at position i
      temp[i] = (int) results.targets_Fiducials[i].fiducialID;
    }

    // return the completed int obj
    return temp;
  }

  /** Outputs if we can see a tag
   @param tag the tag to check to see if we can see
   @apiNote will return null if it cannot get data
   */
  public static boolean CanSeeTag(int tag) {

    // change the pipelime to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the latest results from the limelight
    LimelightResults results = LimelightHelpers.getLatestResults(name);

    // if the limelights intel is bad return false
    if (!results.valid) {
      return false;
    }

    // loop through every tag we can see
    for (LimelightTarget_Fiducial SeenTag : results.targets_Fiducials) {

      // if that tag is the one we are looking for return true
      if (SeenTag.fiducialID == tag) {
        return true;
      }
    }

    // if the tag doesn't appear in the data then return false
    return false;
  }

  /** Gets the pose of the robot in field space based on the tags that we can see
   @apiNote will return null if it cannot find location
  */
  public static Pose2d getRobotPoseInFieldSpace() {

    // Check to see if we are in the sim
    // when in the sim we cannot use limelight and thus should rely on odometry
    if (!Robot.isSimulation()) {

      // get the latest results from the limelight
      LimelightHelpers.setPipelineIndex(name, 0);

      // get the position of the robot on the field in [X, Y]
      double[] botPose = LimelightHelpers.getBotPose(name);

      // make sure the given position is valid
      if (botPose.length != 0){

        // if the position fails to contain good data return null
        if (botPose[0] == 0){
            return null;
        }

        // convert double [x,y] to a pose position
        return new Pose2d(
          new Translation2d(
              botPose[0] + 8.7736 ,
              botPose[1] + 4.0257),
          new Rotation2d(
            Math.toRadians(botPose[5]))
          );
      }

      // if the position fails to contain good data return null
      return null;

    } else {
      // don't pollute odometry
      return null;
    }
  }

  /** Gets the location of the HUB relative to us in 2D space
   @return X, Y of the HUB in robot space / relative to us
   @apiNote will output -1, -1 if it cannot find location
   */
  public static double[] getHUBLocation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int Htag : HUBTags) {
        if (tag.fiducialID == Htag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }
    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  /** Gets the location of the Trench relative to us in 2D space
   @return X, Y of the Trench in robot space / relative to us
   @apiNote will output -1, -1 if it cannot find location
   */
  public static double[] getTrenchLocation() {

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int Htag : trenchTags) {
        if (tag.fiducialID == Htag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }
    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  /** Gets the location of the Outpost relative to us in 2D space
   @return X, Y of the Outpost in robot space / relative to us
   @apiNote will output -1, -1 if it cannot find location
   */
  public static double[] getOutpostLocation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int Htag : outpostTags) {
        if (tag.fiducialID == Htag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }
    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  /** Gets the location of the Tower relative to us in 2D space
   @return X, Y of the Tower in robot space / relative to us
   @apiNote will output -1, -1 if it cannot find location
   */
  public static double[] getTowerLocation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int Htag : towerTags) {
        if (tag.fiducialID == Htag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }
    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  /** Gets the location of the HUB relative to us / in robot space
   @apiNote will return null if it cannot find location
  */
  public static Pose3d getHUBLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int Htag : HUBTags) {
        if (tag.fiducialID == Htag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }
    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  /** Gets the location of the Trench relative to us / in robot space
   @apiNote will return null if it cannot find location
  */
  public static Pose3d getTrenchLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int Htag : trenchTags) {
        if (tag.fiducialID == Htag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }
    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  /** Gets the location of the Outpost relative to us / in robot space
   @apiNote will return null if it cannot find location
  */
  public static Pose3d getOutpostLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int processorTag : outpostTags) {
        if (tag.fiducialID == processorTag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }

    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  /** Gets the location of the Tower relative to us / in robot space
   @apiNote will return null if it cannot find location
  */
  public static Pose3d getTowerLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is bad return null
    if (!results.valid) {
      return null;
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the
      for (int processorTag : towerTags) {
        if (tag.fiducialID == processorTag) {

          // if we have found a tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {

        // continue to break out if we have a tag
        break;
      }
    }

    // if the view of the limelight has no tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  /** Gets the distance between us and the closet Apriltag on the HUB
   @apiNote will return -1 if it cannot find a tag
  */
  public static double DistanceToHUB() {
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

      // loop through all tags to find if this is a tag
      for (int i = 0; i < HUBTags.length; i++) {

        // if it is then make it the new shortest
        if (id == HUBTags[i]) {
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

  /** Gets the distance between us and the closet Apriltag on the Trench
   @apiNote will return -1 if it cannot find a tag
  */
  public static double DistanceToTrench() {
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
      for (int i = 0; i < trenchTags.length; i++) {

        // if it is then make it the new shortest
        if (id == trenchTags[i]) {
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

  /** Gets the distance between us and the closet Apriltag on the Outpost
   @apiNote will return -1 if it cannot find a tag
  */
  public static double DistanceToOutpost() {
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
      for (int i = 0; i < outpostTags.length; i++) {

        // if it is then make it the new shortest
        if (id == outpostTags[i]) {
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

  /** Gets the distance between us and the closet Apriltag on the Tower
   @apiNote will return -1 if it cannot find a tag
  */
  public static double DistanceToTower() {
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
      for (int i = 0; i < towerTags.length; i++) {

        // if it is then make it the new shortest
        if (id == towerTags[i]) {
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
