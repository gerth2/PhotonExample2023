// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;


public class PhotonCameraWrapper {

  public PhotonCamera ovCam;
  public PhotonCamera usbCam;
  public RobotPoseEstimator rpe;

  public PhotonCameraWrapper() {

    //Set up a test arena of two apriltags at the center of each driver station set
    final double fl_m = Units.feetToMeters(54);
    final double fw_m = Units.feetToMeters(27);
    final AprilTag tag18 = new AprilTag(18, new Pose3d(new Pose2d(fl_m,  fw_m/2.0, Rotation2d.fromDegrees(180))));
    final AprilTag tag01 = new AprilTag(01, new Pose3d(new Pose2d(0.0, fw_m/2.0, Rotation2d.fromDegrees(0.0))));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag18);
    atList.add(tag01);

    //TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
    AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, fl_m, fw_m);

    //Forward Camera
    ovCam = new PhotonCamera("OV9281");
    Transform3d robotToOvCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    usbCam = new PhotonCamera("USB_Camera");
    Transform3d robotToUsbCam = new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0,0,Math.PI)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    // ... Add other cameras here

    // Assemble the list of cameras & mount locations
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(ovCam, robotToOvCam));
    camList.add(new Pair<PhotonCamera, Transform3d>(usbCam, robotToUsbCam));

    rpe = new RobotPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time of the observation.
   * Assumes a planar field and the robot is always firmly on the ground
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    rpe.setReferencePose(prevEstimatedRobotPose);
    
    var rxTime = Timer.getFPGATimestamp();
    var result = rpe.update();
    if(result.getFirst() != null){
      return new Pair<Pose2d, Double>(result.getFirst().toPose2d(), rxTime - result.getSecond());
    } else {
      return new Pair<Pose2d, Double>(null, 0.0);
    }

  }
}
