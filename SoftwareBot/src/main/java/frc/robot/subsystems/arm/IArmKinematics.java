package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public interface IArmKinematics {

    // Forward kinematics - Convert from C-space to world space.
    public Pose2d forward(ArmState state);

    // Inverse kinematics - Convert from world space to C-space.
    public ArmState inverse(Pose2d pose);
}