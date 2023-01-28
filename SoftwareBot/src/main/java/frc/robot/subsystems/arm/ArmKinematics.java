package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public class ArmKinematics implements IArmKinematics {

    // Forward kinematics - Convert from C-space to world space.
    public Pose2d forward(ArmState state) {
        return new Pose2d();
    }

    // Inverse kinematics - Convert from world space to C-space.
    public ArmState inverse(Pose2d pose) {
        return new ArmState();
    }
}