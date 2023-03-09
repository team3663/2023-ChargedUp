package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // This flag tells us whether to configure for a competition match (as opposed to practice)
    // We mostly use it to disable diagnostics that should not run at competition.
    public static final boolean COMPETITION_MODE = false;

    public static class ControllerPorts {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
        public static final int TEST = 2;
    }

    public static class DioIds {
        public static final int RED_LED_ID = 0;
        public static final int GREEN_LED_ID = 1;
        public static final int BLUE_LED_ID = 2;

        public static final int ARM_SHOULDER_MOTOR = 1;
        public static final int ARM_ELBOW_MOTOR = 2;
        public static final int ARM_WRIST_MOTOR = 3;

        public static final int ARM_SHOULDER_ENCODER = 4;
        public static final int ARM_ELBOW_ENCODER = 5;
        public static final int ARM_WRIST_ENCODER = 6;
    }

    public static class CameraPoses {
        public static final Pose3d ARDUCAM_POSE = new Pose3d(
            -Units.inchesToMeters(10.5),
            Units.inchesToMeters(11.75),
            Units.inchesToMeters(30),
            new Rotation3d(0, 0, Math.PI / 2)
        );
    }
}
