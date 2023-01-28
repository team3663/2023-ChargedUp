package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public interface IArmKinematics {

    // Forward kinematics - Convert from C-space to task space.
    public Pose2d forward(ArmState state);

    // Inverse kinematics - Convert from task space to C-space.
    public ArmState inverse(Pose2d pose);
}