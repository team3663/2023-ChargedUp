package frc.robot.photonvision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public interface IPhotonVision {
 
    default public void update() {}
    default public void setReferencePose(Pose2d pose) {}
    default public Optional<EstimatedRobotPose> getRobotPose3d() { return null; }
}
