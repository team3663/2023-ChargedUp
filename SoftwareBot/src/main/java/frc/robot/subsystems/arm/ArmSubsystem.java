package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;

import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

    // Lengths of arms three linkages (arm, forearm & hand)
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(29.25);
    private static final double FOREARM_LENGTH_METERS = Units.inchesToMeters(27.5);
    private static final double HAND_LENGTH_METERS = Units.inchesToMeters(17);

    // angle constraints for each joint
    private static final double SHOULDER_MIN_ANGLE_RAD = Units.degreesToRadians(91);
    private static final double SHOULDER_MAX_ANGLE_RAD = Units.degreesToRadians(155);
    private static final double ELBOW_MIN_ANGLE_RAD = Units.degreesToRadians(-173);
    private static final double ELBOW_MAX_ANGLE_RAD = Units.degreesToRadians(-3);
    private static final double WRIST_MIN_ANGLE_RAD = Units.degreesToRadians(-90);
    private static final double WRIST_MAX_ANGLE_RAD = Units.degreesToRadians(135);

    // While the elbow is inside the danger zone, the wrist must stay within its safety zone to prevent damage.
    private static final double ELBOW_DANGER_ZONE_RAD = Units.degreesToRadians(-130);
    private static final double WRIST_SAFETY_ANGLE_RAD = Units.degreesToRadians(45);

    // private static final double ELBOW_VELOCITY_CONSTANT = 1.6;
    private static final double ELBOW_GRAVITY_GAIN_COEFFICIENT = 0.25;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private IArmKinematics kinematics;
    private Mechanism2d mechanism;

    // Initial target pose, this should always be our stowed position.
    private Pose2d targetPose = ArmPoseLibrary.get(ArmPoseID.STOWED);

    // Arms current target state
    ArmState targetState;

    private double[] logBuffer = new double[3];

    private final ArmLinkage arm = new ArmLinkage(ARM_LENGTH_METERS, SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD);
    private final ArmLinkage forearm = new ArmLinkage(FOREARM_LENGTH_METERS, ELBOW_MIN_ANGLE_RAD, ELBOW_MAX_ANGLE_RAD);
    private final ArmLinkage hand = new ArmLinkage(HAND_LENGTH_METERS, WRIST_MIN_ANGLE_RAD, WRIST_MAX_ANGLE_RAD);

    private final MechanismLigament2d currentArmLigament = new MechanismLigament2d("CurrentArm", ARM_LENGTH_METERS, 90.0, 10, new Color8Bit(0, 0, 255));
    private final MechanismLigament2d currentForearmLigament = new MechanismLigament2d("CurrentForearm", FOREARM_LENGTH_METERS, 0.0, 10, new Color8Bit(0, 255, 0));
    private final MechanismLigament2d currentIntakeLigament = new MechanismLigament2d("CurrentIntake", HAND_LENGTH_METERS, 0.0, 10, new Color8Bit(255, 255, 0));

    private final MechanismLigament2d targetArmLigament = new MechanismLigament2d("TargetArm", ARM_LENGTH_METERS, 90.0, 10, new Color8Bit(0, 0, 128));
    private final MechanismLigament2d targetForearmLigament = new MechanismLigament2d("TargetForearm", FOREARM_LENGTH_METERS, 0.0, 10, new Color8Bit(0, 128, 0));
    private final MechanismLigament2d targetIntakeLigament = new MechanismLigament2d("TargetIntake", HAND_LENGTH_METERS, 0.0, 10, new Color8Bit(128, 128, 0));

    private final MechanismRoot2d targetPositionRoot;
    private final MechanismLigament2d targetPositionLigament;
    private final Color8Bit validStateColor = new Color8Bit(255, 255, 255);
    private final Color8Bit invalidStateColor = new Color8Bit(255, 0, 0);

    private final PIDController shoulderController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController elbowController = new PIDController(2.7, 0.0, 0.0);
    private final PIDController wristController = new PIDController(1.0, 0.0, 0.0);

    public ArmSubsystem(ArmIO io) {
        this.io = io;

        // Create arm kinematics and use it to calculate initial target state
        this.kinematics = new ArmKinematics(arm, forearm, hand);
        targetState = kinematics.inverse(targetPose);

        // Create the mechanism object with a 4M x 3M canvas
        this.mechanism = new Mechanism2d(4, 3);
        MechanismRoot2d root = mechanism.getRoot("shoulder", 2, 1);
        root.append(currentArmLigament);
        currentArmLigament.append(currentForearmLigament);
        currentForearmLigament.append(currentIntakeLigament);

        root.append(targetArmLigament);
        targetArmLigament.append(targetForearmLigament);
        targetForearmLigament.append(targetIntakeLigament);

        targetPositionRoot = mechanism.getRoot("target", 0, 0);
        targetPositionLigament = new MechanismLigament2d("TargetPosition", HAND_LENGTH_METERS, 0, 10, new Color8Bit(255, 255, 255));
        targetPositionRoot.append(targetPositionLigament);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm/Inputs", inputs);

        // Calculate a gravity gain value that we use to scale output of the elbow controller based on the angle of the forearm
        // This helps to compensate for the change in moment of the forearm as its angle changes relative to the floor by increasing
        // the applied voltage for the elbow joint the closer to horizontal it is.
        double elbowGravityGain = 1 + Math.abs(Math.cos(inputs.shoulderAngleRad + inputs.elbowAngleRad)) * ELBOW_GRAVITY_GAIN_COEFFICIENT;
        Logger.getInstance().recordOutput("Arm/ElbowGravityGain", elbowGravityGain);

        double targetWristAngle = targetState.wristAngleRad;
        double targetElbowAngle = targetState.elbowAngleRad;
        double targetShoulderAngle = MathUtil.clamp(targetState.shoulderAngleRad, SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD);

        if (inputs.elbowAngleRad < ELBOW_DANGER_ZONE_RAD || targetElbowAngle < ELBOW_DANGER_ZONE_RAD) {
            targetWristAngle = Math.max(targetWristAngle, WRIST_SAFETY_ANGLE_RAD);
        }
        if (inputs.wristAngleRad < WRIST_SAFETY_ANGLE_RAD && inputs.elbowAngleRad > ELBOW_DANGER_ZONE_RAD) {
            targetElbowAngle = Math.max(targetState.elbowAngleRad, ELBOW_DANGER_ZONE_RAD);
        }

        double shoulderVoltage = shoulderController.calculate(inputs.shoulderAngleRad, targetShoulderAngle);
        double elbowVoltage = elbowController.calculate(inputs.elbowAngleRad, targetElbowAngle) * elbowGravityGain;
        double wristVoltage = wristController.calculate(inputs.wristAngleRad, targetWristAngle);

        // TODO: Remove this
        // shoulderVoltage = MathUtil.clamp(shoulderVoltage, -2, 2);
        // elbowVoltage = MathUtil.clamp(elbowVoltage, -2, 2);
        // wristVoltage = MathUtil.clamp(wristVoltage, -2, 2);
        
        io.setShoulderVoltage(applyJointLimits(shoulderVoltage, inputs.shoulderAngleRad, SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD));
        io.setElbowVoltage(applyJointLimits(elbowVoltage, inputs.elbowAngleRad, ELBOW_MIN_ANGLE_RAD, ELBOW_MAX_ANGLE_RAD));
        io.setWristVoltage(applyJointLimits(wristVoltage, inputs.wristAngleRad, WRIST_MIN_ANGLE_RAD, WRIST_MAX_ANGLE_RAD));   

        // Log current joint voltages
        logBuffer[0] = inputs.shoulderAppliedVoltage;
        logBuffer[1] = inputs.elbowAppliedVoltage;
        logBuffer[2] = inputs.wristAppliedVoltage;
        Logger.getInstance().recordOutput("Arm/JointVoltages", logBuffer);

        // Log target pose and arm state
        logBuffer[0] = targetState.shoulderAngleRad;
        logBuffer[1] = targetState.elbowAngleRad;
        logBuffer[2] = targetState.wristAngleRad;
        Logger.getInstance().recordOutput("Arm/TargetPose", targetPose);
        Logger.getInstance().recordOutput("Arm/TargetState", logBuffer);

        currentArmLigament.setAngle(Units.radiansToDegrees(inputs.shoulderAngleRad));
        currentForearmLigament.setAngle(Units.radiansToDegrees(inputs.elbowAngleRad));
        currentIntakeLigament.setAngle(Units.radiansToDegrees(inputs.wristAngleRad));

        targetPositionRoot.setPosition(targetPose.getX() + 2, targetPose.getY() + 1);
        targetPositionLigament.setAngle(targetPose.getRotation().getDegrees());

        // If the targetPose resulted in a valid arms state then set color of the targetPositionLigament
        // to white, for invalid states make it red.
        targetPositionLigament.setColor(targetState.valid ? validStateColor : invalidStateColor);

        targetArmLigament.setAngle(Units.radiansToDegrees(targetState.shoulderAngleRad));
        targetForearmLigament.setAngle(Units.radiansToDegrees(targetState.elbowAngleRad));
        targetIntakeLigament.setAngle(Units.radiansToDegrees(targetState.wristAngleRad));
        Logger.getInstance().recordOutput("Arm/Mechanism", mechanism);
        
        // Log current state and pose out to AdvantateKit
        ArmState currentState = new ArmState(inputs.shoulderAngleRad, inputs.elbowAngleRad, inputs.wristAngleRad);
        Pose2d currentPose = kinematics.forward(currentState);  
        logBuffer[0] = currentState.shoulderAngleRad;
        logBuffer[1] = currentState.elbowAngleRad;
        logBuffer[2] = currentState.wristAngleRad;
        Logger.getInstance().recordOutput("Arm/CurrentPose", currentPose);
        Logger.getInstance().recordOutput("Arm/CurrentState", logBuffer);
    }

    private double applyJointLimits(double voltage, double currentAngle, double minAngle, double maxAngle) {

        if (currentAngle > maxAngle) {
            return Math.min(voltage, 0);
        }
        
        if (currentAngle < minAngle) {
            return Math.max(voltage, 0);
        }

        return voltage;
    }

    public void setPose(Pose2d targetPose) {
        this.targetPose = targetPose;

        // Calculate desired target state from the new target pose.
        targetState = kinematics.inverse(targetPose);
    }

    public Pose2d getPose() {
        return targetPose;
    }
}
