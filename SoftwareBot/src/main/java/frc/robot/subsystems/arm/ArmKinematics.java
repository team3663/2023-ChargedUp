/*
 * Kinematics for our robot arm.
 * 
 * Based on information from the CMU lecture notes and Robot Academy video listed below.
 * 
 * https://www.cs.cmu.edu/~motionplanning/lecture/Chap3-Config-Space_howie.pdf
 * 
 * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
 * 
 */
package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmKinematics implements IArmKinematics {

    private final ArmLinkage arm;
    private final ArmLinkage forearm;
    private final ArmLinkage hand;


    public ArmKinematics(ArmLinkage arm, ArmLinkage forearm, ArmLinkage hand) {
        this.arm = arm;
        this.forearm = forearm;
        this.hand = hand;
    }

    /**
     * Forward kinematics - Convert from C-space to task space.
     * 
     * @param state - ArmState object containing a set of joint angles (c-space) for the arm.
     * 
     * @return Pose2d object containing the arm pose (task space) that will result from the provided arm configuration.
     */
    @Override
    public Pose2d forward(ArmState state) {
        double x = arm.lengthMeters * Math.cos(state.shoulderAngleRad) + forearm.lengthMeters * Math.cos(state.shoulderAngleRad + state.elbowAngleRad);
        double y = arm.lengthMeters * Math.sin(state.shoulderAngleRad) + forearm.lengthMeters * Math.sin(state.shoulderAngleRad + state.elbowAngleRad);

        // Determine the hand angle (relative to x axis)
        double handAngle = state.shoulderAngleRad + state.elbowAngleRad + state.wristAngleRad;

        return new Pose2d(x, y, new Rotation2d(handAngle));
    }

    /**
     * Inverse kinematics - Convert from task space to C-space.
     * 
     * @param pose - Desired pose (task space) for the robot arm.
     * 
     * @return ArmState object conatining joint angles (c-space) that will yield the desired pose.
     */
    @Override    
    public ArmState inverse(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double handAngle = pose.getRotation().getRadians();
    
        // Calculate the elbow angle, there are two possible symetrical solutions for the elbow angle, we choose the negative one since
        // it gives us poses using higher shoulder angles, which we prefer.
        double elbowAngle = -Math.acos((x * x + y * y - arm.lengthMeters * arm.lengthMeters - forearm.lengthMeters * forearm.lengthMeters) / (2 * arm.lengthMeters * forearm.lengthMeters));

        // Calculate the shoulder angle based on the elbow angle we just found.
        double shoulderAngle = Math.atan2(y,x) - Math.atan2(forearm.lengthMeters * Math.sin(elbowAngle), arm.lengthMeters + forearm.lengthMeters * Math.cos(elbowAngle));

        // Calculate the angle of the forearm, this is the sum of the shoulder and elbow angles.
        double forearmAngle = shoulderAngle + elbowAngle;

        // Now calculate the wrist angle that gives the desired hand angle based on the forearm angle we just calculated
        double wristAngle = handAngle - forearmAngle;

        Logger.getInstance().recordOutput("Arm/rawShoulderAngle", shoulderAngle);
        Logger.getInstance().recordOutput("Arm/rawElbowAngle", elbowAngle);
        Logger.getInstance().recordOutput("Arm/rawWristAngle", wristAngle);

        double shoulderAngleClamped = MathUtil.clamp(shoulderAngle, arm.minAngleRad, arm.maxAngleRad);
        double elbowAngleClamped = MathUtil.clamp(elbowAngle, forearm.minAngleRad, forearm.maxAngleRad);
        double wristAngleClamped = MathUtil.clamp(wristAngle, hand.minAngleRad, hand.maxAngleRad);

        if (shoulderAngleClamped != shoulderAngle) {
            System.out.println("Invalid shoulder angle");
        }
        if (elbowAngleClamped != elbowAngle) {
            System.out.println("Invalid elbow angle");
        }
        if (wristAngleClamped != wristAngle) {
            System.out.println("Invalid wrist angle");
        }
    
        Logger.getInstance().recordOutput("Arm/kinematicsShoulderAngle", shoulderAngle);
        Logger.getInstance().recordOutput("Arm/kinematicsElbowAngle", elbowAngle);
        Logger.getInstance().recordOutput("Arm/kinematicsWristAngle", wristAngle);

        Logger.getInstance().recordOutput("Arm/clampedShoulderAngle", shoulderAngleClamped);
        Logger.getInstance().recordOutput("Arm/clampedElbowAngle", elbowAngleClamped);
        Logger.getInstance().recordOutput("Arm/clampedWristAngle", wristAngleClamped);

        return new ArmState(shoulderAngleClamped, elbowAngleClamped, wristAngleClamped);
      }
}
