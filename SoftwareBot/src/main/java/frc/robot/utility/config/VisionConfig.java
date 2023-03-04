package frc.robot.utility.config;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.photonvision.IPhotonVision;
import frc.robot.photonvision.PhotonVisionUtil;
import lombok.Data;
import org.photonvision.PhotonCamera;

import java.util.ArrayList;
import java.util.List;

@Data
public class VisionConfig {
    private List<CameraDescriptor> cameras = new ArrayList<>();

    private static final double cx = Units.inchesToMeters(4.5);
    private static final double cy = Units.inchesToMeters(8.5);
    private static final double cz = Units.inchesToMeters(25.5);

    public IPhotonVision create() {
        // TODO: Verify camera poses
        cameras.add(
            new CameraDescriptor("Left_Camera",
            new Transform3d(new Pose3d(), new Pose3d(-cx, cy, cz, new Rotation3d(0, 0, Units.degreesToRadians(30)))))
        );
        cameras.add(
            new CameraDescriptor("Right_Camera",
            new Transform3d(new Pose3d(), new Pose3d(-cx, -cy, cz, new Rotation3d(0, 0, Units.degreesToRadians(-30)))))
        );

        PhotonCamera[] photonCameras = new PhotonCamera[cameras.size()];
        Transform3d[] cameraTransforms = new Transform3d[cameras.size()];

        for (int i = 0; i < cameras.size(); ++i) {
            photonCameras[i] = new PhotonCamera(cameras.get(i).getName());
            cameraTransforms[i] = cameras.get(i).getTransform();
        }

        return new PhotonVisionUtil(photonCameras, cameraTransforms);
    }

    @Data
    public static class CameraDescriptor {
        private String name;
        private Transform3d transform;

        public CameraDescriptor(String name, Transform3d transform) {
            this.name = name;
            this.transform = transform;
        }
    }
}
