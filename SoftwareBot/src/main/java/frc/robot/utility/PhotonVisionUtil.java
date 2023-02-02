// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;  
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.util.Units;

public class PhotonVisionUtil extends SubsystemBase {

  private GenericEntry hasTargets;
  private GenericEntry targetID;
  private GenericEntry targetX;
  private GenericEntry targetY;
  private GenericEntry targetZ;
  private GenericEntry robotX;
  private GenericEntry robotY;
  private GenericEntry robotTheta;
  private GenericEntry targetAmbiguity;

  private boolean n_hasTargets;
  private int n_targetID;
  private double n_targetX;
  private double n_targetY;
  private double n_targetZ;
  private double n_robotX;
  private double n_robotY;
  private double n_robotTheta;
  private double n_targetAmbiguity;

  private final PhotonCamera[] cameras;
  private final Transform3d[] cameraPoses;

  private final AprilTagFieldLayout layout;
  private final Path fieldJsonPath = Paths.get(Filesystem.getDeployDirectory().toString(), "MS-Atrium.json");
  private final ArrayList<Pair<PhotonCamera, Transform3d>> cameraList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

  private final ArrayList<PhotonPoseEstimator> poseEstimators = new ArrayList<PhotonPoseEstimator>();
  private ArrayList<PhotonPipelineResult> pipelineResults = new ArrayList<PhotonPipelineResult>();

  private Optional<EstimatedRobotPose> estPose;
  private Pose3d robotPose;

  private boolean targetAcquired;
  private PhotonTrackedTarget chosenTarget;

  /** Creates a new VisionSubsystem. */
  public PhotonVisionUtil(PhotonCamera[] cameras, Transform3d[] cameraPoses) {
    this.cameras = new PhotonCamera[cameras.length];
    this.cameraPoses = new  Transform3d[cameraPoses.length];
    for (int i = 0; i < cameras.length; i++) {
      this.cameras[i] = cameras[i];
      this.cameraPoses[i] = cameraPoses[i];
      cameraList.add(new Pair<PhotonCamera, Transform3d>(cameras[i], cameraPoses[i]));
    }

    try {
      layout = new AprilTagFieldLayout(fieldJsonPath);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    robotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    // for (PhotonCamera c : cameras) {
    for (int i = 0; i < cameras.length; i++) {
      PhotonPoseEstimator pe = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, cameras[i], cameraPoses[i]);
      poseEstimators.add(pe);
    }

    initTelemetry();
  }

  @Override
  public void periodic() {
    int missCounter = 0;

    for (PhotonCamera c : cameras) {
      PhotonPipelineResult pr = c.getLatestResult();
      pipelineResults.add(pr);
    }

    for (int i = 0; i < pipelineResults.size(); i++) {
      if (pipelineResults.get(i).hasTargets()) {
        estPose = poseEstimators.get(i).update();
        chosenTarget = pipelineResults.get(i).getBestTarget();
        targetAcquired = true;
      } else {
        missCounter++;
      }
    }

    // TODO: Make sure this actually works; It probably won't
    if  (missCounter == pipelineResults.size()) {
      targetAcquired = false;
    }

    if (targetAcquired) {
      n_targetID = chosenTarget.getFiducialId();
      n_targetX = processDistance(chosenTarget.getBestCameraToTarget().getX());
      n_targetY = processDistance(chosenTarget.getBestCameraToTarget().getY());
      n_targetZ = processDistance(chosenTarget.getBestCameraToTarget().getZ());
      n_targetAmbiguity = chosenTarget.getPoseAmbiguity();
      n_robotX = processDistance(robotPose.getX());
      n_robotY = processDistance(robotPose.getY());
      n_robotTheta = ((double) Math.round(robotPose.getRotation().toRotation2d().getDegrees() * 100)) / 100;

      robotPose = estPose.get().estimatedPose;
    } else {
      n_targetID = 0;
      n_targetX = 0;
      n_targetY = 0;
      n_targetZ = 0;
      n_targetAmbiguity = 0;
      n_robotX = 0;
      n_robotY = 0;
      n_robotTheta = 0;
    }
    updateTelemetry();
  }

  private double processDistance (double dist) {
    double dp = Units.metersToInches(dist);
    dp = (double) Math.round(dp *= 100);
    dp /= 100;    

    return dp;
  }

  public EstimatedRobotPose getRobotPose3d () {
    boolean hasValidPose = false;
    int g = -1;
    for (int i = 0; i < poseEstimators.size(); i++) {
        if (pipelineResults.get(i).hasTargets()) {
            hasValidPose = true;
            g = i;
        }
    }
    if (hasValidPose) {
        return poseEstimators.get(g).update().get();
    } else {
        return new EstimatedRobotPose(new Pose3d(), 0);
    }
  }

  private void initTelemetry () {
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    hasTargets = visionTab.add("Target(s) Acquired", false)
      .withPosition(0, 0)
      .withSize(1, 1)
      .getEntry();

    targetID = visionTab.add("Target ID", 0)
      .withPosition(1, 0)
      .withSize(1, 1)
      .getEntry();
    
    targetX = visionTab.add("Target X", 0.0)
      .withPosition(2, 0)
      .withSize(1, 1)
      .getEntry();

    targetY = visionTab.add("Target Y", 0.0)
      .withPosition(3, 0)
      .withSize(1, 1)
      .getEntry();

    targetZ = visionTab.add("Target Z", 0.0)
      .withPosition(4, 0)
      .withSize(1, 1)
      .getEntry();

    targetAmbiguity = visionTab.add("Target Ambiguity", 0.0)
      .withPosition(5, 0)
      .withSize(1, 1)
      .getEntry();

    robotX = visionTab.add("Robot xPos", 0.0)
      .withPosition(0, 1)
      .withSize(1, 1)
      .getEntry();

    robotY = visionTab.add("Robot yPos", 0.0)
      .withPosition(1, 1)
      .withSize(1, 1)
      .getEntry();

    robotTheta = visionTab.add("Robot Angle", 0.0)
      .withPosition(2, 1)
      .withSize(1, 1)
      .getEntry();
  }

  private void updateTelemetry () {
    hasTargets.setBoolean(n_hasTargets);
    targetID.setValue(n_targetID);
    targetX.setValue(n_targetX);
    targetY.setValue(n_targetY);
    targetZ.setValue(n_targetZ);
    targetAmbiguity.setValue(n_targetAmbiguity);
    robotX.setValue(n_robotX);
    robotY.setValue(n_robotY);
    robotTheta.setValue(n_robotTheta);
  }
}
