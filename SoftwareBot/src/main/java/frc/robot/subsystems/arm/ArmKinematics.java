package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public class ArmKinematics implements IArmKinematics {

    // Forward kinematics - Convert from C-space to task space.
    public Pose2d forward(ArmState state) {
        //TODO: Provide real implementation
        return new Pose2d();
    }

    // Inverse kinematics - Convert from task space to C-space.
    public ArmState inverse(Pose2d pose) {
        //TODO: Provide real implementation
        return new ArmState();
    }
}