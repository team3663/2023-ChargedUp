// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(131.5);// 311.75
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(130.6);// 311.92
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(95.3);// 275.27
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(112.5);// 285.03

    public static class ControllerPorts {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
        public static final int TEST = 2;
    }

    public static class CanIds {
        public static final int DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
        public static final int DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
        public static final int DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;

        public static final int DRIVETRAIN_FRONT_LEFT_MODULE_STEER_MOTOR = 5;
        public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
        public static final int DRIVETRAIN_BACK_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int DRIVETRAIN_BACK_RIGHT_MODULE_STEER_MOTOR = 8;

        public static final int DRIVETRAIN_FRONT_LEFT_MODULE_STEER_ENCODER = 9;
        public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
        public static final int DRIVETRAIN_BACK_LEFT_MODULE_STEER_ENCODER = 11;
        public static final int DRIVETRAIN_BACK_RIGHT_MODULE_STEER_ENCODER = 12;

        public static final int DRIVETRAIN_PIGEON_ID = 13;
    }

    public static class CameraPoses {
        public static final Pose3d ARDUCAM_POSE = new Pose3d(
            -Units.inchesToMeters(10.5),
            Units.inchesToMeters(11.75),
            Units.inchesToMeters(30),
            new Rotation3d(0, 0, Math.PI / 2)
            // new Rotation3d(0, 0, 0.5)
        );
        public static final Pose3d RIGHT_CAMERA_POSE = new Pose3d(
            -Units.inchesToMeters(10.5),
            -Units.inchesToMeters(11.75),
            Units.inchesToMeters(30),
            new Rotation3d(0, 0, -Math.PI / 2)
            // new Rotation3d(0, 0, -0.5)
        );
    }
}
