package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmKinematics implements IArmKinematics {

    private double[] armLengthConstants;
    private double[] armAngleConstraints;

    public ArmKinematics (double[] armLengthConstants, double[] armAngleConstraints) {
        this.armLengthConstants = armLengthConstants;
        this.armAngleConstraints = armAngleConstraints;
    }

    // Forward kinematics - Convert from C-space to task space.
    public Pose2d forward(ArmState state) {
        double xPos = 0;
        double yPos = 0;

        xPos = (armLengthConstants[0] * Math.cos(state.shoulderAngleRad)) + (armLengthConstants[1] * Math.cos(state.elbowAngleRad));
        yPos = (armLengthConstants[0] * Math.sin(state.shoulderAngleRad)) + (armLengthConstants[1] * Math.sin(state.elbowAngleRad));

        return new Pose2d(new Translation2d(xPos, yPos), new Rotation2d(state.wristAngleRad));
    }

    // Inverse kinematics - Convert from task space to C-space.
    public ArmState inverse(Pose2d pose) {
        ArmState state = new ArmState();

        double IKShoulderAngleRad = armAngleConstraints[0];
        double IKElbowAngleRad = armAngleConstraints[2];

        // This code is a mess
        IKElbowAngleRad = 1 / Math.cos(
            (Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2) - Math.pow(armLengthConstants[0], 2) - Math.pow(armLengthConstants[1], 2))
            /
            (2 * armLengthConstants[0] * armLengthConstants[1])
        );

        IKShoulderAngleRad = 1 / Math.sin((armLengthConstants[1] * Math.sin(IKElbowAngleRad)) / (pose.getX() * pose.getY()))
            +
            Math.atan2(pose.getY(), pose.getX());
        
        if (IKShoulderAngleRad >= armAngleConstraints[0] || IKShoulderAngleRad <= armAngleConstraints[1] ||
                IKElbowAngleRad >= armAngleConstraints[2] || IKShoulderAngleRad <= armAngleConstraints[3] ||
                pose.getRotation().getRadians() >= armAngleConstraints[4] || pose.getRotation().getRadians() <= armAngleConstraints[5])
        {
            state.elbowAngleRad = IKElbowAngleRad;
            state.shoulderAngleRad = IKShoulderAngleRad;
            state.wristAngleRad = pose.getRotation().getRadians();
            return state;
        } else {
            System.out.println("Invalid Arm Position");
            return new ArmState();
        }
    }
}