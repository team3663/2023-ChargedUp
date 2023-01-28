package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public interface IArmKinematics {

    public Pose2d armStateToPose(ArmState state);

    public ArmState poseToArmState(Pose2d pose);
}