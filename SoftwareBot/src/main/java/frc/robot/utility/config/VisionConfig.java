package frc.robot.utility.config;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.photonvision.IPhotonVision;
import frc.robot.photonvision.PhotonVisionUtil;
import lombok.Data;
import org.photonvision.PhotonCamera;

import java.util.ArrayList;
import java.util.List;

@Data
public class VisionConfig {
    private List<CameraDescriptor> cameras = new ArrayList<>();

    public IPhotonVision create() {
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
    }
}
