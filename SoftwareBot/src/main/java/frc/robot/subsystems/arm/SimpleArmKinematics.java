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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimpleArmKinematics implements IArmKinematics {

    private final ArmLinkage arm;
    private final ArmLinkage forearm;
    @SuppressWarnings("unused")
    private final ArmLinkage hand;
    private final double[] constraints;


    public SimpleArmKinematics(ArmLinkage arm, ArmLinkage forearm, ArmLinkage hand, double[] constraints) {
        this.arm = arm;
        this.forearm = forearm;
        this.hand = hand;
        this.constraints = constraints;
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

        double shoulderAngleRadActual = MathUtil.clamp(state.elbowAngleRad, constraints[0], constraints[1]);
        double elbowAngleRadActual = MathUtil.clamp(state.shoulderAngleRad, constraints[2], constraints[3]);
        double wristAngleRadActual = MathUtil.clamp(state.wristAngleRad, constraints[4], constraints[5]);

        if (shoulderAngleRadActual != state.shoulderAngleRad) {
            System.out.println("Invalid shoulder angle");
        }
        if (elbowAngleRadActual != state.elbowAngleRad) {
            System.out.println("Invalid elbow angle");
        }
        if (wristAngleRadActual != state.wristAngleRad) {
            System.out.println("Invalid wrist angle");
        }
        
        double x = arm.lengthMeters * Math.cos(shoulderAngleRadActual) + forearm.lengthMeters * Math.cos(shoulderAngleRadActual + elbowAngleRadActual);
        double y = arm.lengthMeters * Math.sin(shoulderAngleRadActual) + forearm.lengthMeters * Math.sin(shoulderAngleRadActual + elbowAngleRadActual);

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

        double shoulderAngleRadActual = MathUtil.clamp(elbowAngle, constraints[0], constraints[1]);
        double elbowAngleRadActual = MathUtil.clamp(shoulderAngle, constraints[2], constraints[3]);
        double wristAngleRadActual = MathUtil.clamp(wristAngle, constraints[4], constraints[5]);

        if (shoulderAngleRadActual != shoulderAngle) {
            System.out.println("Invalid shoulder angle");
        }
        if (elbowAngleRadActual != elbowAngle) {
            System.out.println("Invalid elbow angle");
        }
        if (wristAngleRadActual != wristAngle) {
            System.out.println("Invalid wrist angle");
        }
    
        return new ArmState(shoulderAngle, elbowAngle, wristAngle);
      }
}
