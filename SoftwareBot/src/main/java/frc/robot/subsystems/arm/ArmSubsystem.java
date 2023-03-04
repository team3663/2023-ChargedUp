package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

    // Lengths of arms three linkages (arm, forearm & hand)
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(29.25);
    private static final double FOREARM_LENGTH_METERS = Units.inchesToMeters(27.5);
    private static final double HAND_LENGTH_METERS = Units.inchesToMeters(17);

    // angle constraints for each joint
    private static final double SHOULDER_MIN_ANGLE_RAD = Units.degreesToRadians(91);
    private static final double SHOULDER_MAX_ANGLE_RAD = Units.degreesToRadians(157.25);
    private static final double ELBOW_MIN_ANGLE_RAD = Units.degreesToRadians(-173);
    private static final double ELBOW_MAX_ANGLE_RAD = Units.degreesToRadians(-3);
    private static final double WRIST_MIN_ANGLE_RAD = Units.degreesToRadians(-1);
    private static final double WRIST_MAX_ANGLE_RAD = Units.degreesToRadians(120);

    // private static final double ELBOW_VELOCITY_CONSTANT = 1.6;
    private static final double ELBOW_BACKLASH_CONSTANT = Units.degreesToRadians(5);
    private static double elbowGravityGain = 0;
    private static final double ELBOW_GRAVITY_GAIN_COEFFICIENT = 0.25;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private Pose2d targetPose = new Pose2d(0.002, 0.16, Rotation2d.fromDegrees(90.0));
    private IArmKinematics kinematics;
    private Mechanism2d mechanism;

    private double[] targetAnglesLogged = new double[3];

    private final ArmLinkage arm = new ArmLinkage(ARM_LENGTH_METERS, SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD);
    private final ArmLinkage forearm = new ArmLinkage(FOREARM_LENGTH_METERS, ELBOW_MIN_ANGLE_RAD, ELBOW_MAX_ANGLE_RAD);
    private final ArmLinkage hand = new ArmLinkage(HAND_LENGTH_METERS, WRIST_MIN_ANGLE_RAD, WRIST_MAX_ANGLE_RAD);

    private final MechanismLigament2d currentArmLigament = new MechanismLigament2d("CurrentArm", ARM_LENGTH_METERS, 90.0, 10, new Color8Bit(0, 0, 255));
    private final MechanismLigament2d currentForearmLigament = new MechanismLigament2d("CurrentForearm", FOREARM_LENGTH_METERS, 0.0, 10, new Color8Bit(0, 255, 0));
    private final MechanismLigament2d currentIntakeLigament = new MechanismLigament2d("CurrentIntake", HAND_LENGTH_METERS, 0.0, 10, new Color8Bit(255, 0, 0));

    private final MechanismLigament2d targetArmLigament = new MechanismLigament2d("TargetArm", ARM_LENGTH_METERS, 90.0, 10, new Color8Bit(0, 0, 128));
    private final MechanismLigament2d targetForearmLigament = new MechanismLigament2d("TargetForearm", FOREARM_LENGTH_METERS, 0.0, 10, new Color8Bit(0, 128, 0));
    private final MechanismLigament2d targetIntakeLigament = new MechanismLigament2d("TargetIntake", HAND_LENGTH_METERS, 0.0, 10, new Color8Bit(128, 0, 0));

    private final MechanismRoot2d targetPositionRoot;
    private final MechanismLigament2d targetPositionLigament;

    private final PIDController shoulderController = new PIDController(30.0, 0.0, 0.0);
    private final PIDController elbowController = new PIDController(2.7, 0.0, 0.0);
    private final PIDController wristController = new PIDController(20.0, 0.0, 0.0);

    public ArmSubsystem(ArmIO io) {
        this.io = io;
        this.kinematics = new ArmKinematics(arm, forearm, hand);

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

        elbowGravityGain = Math.abs(Math.cos(inputs.shoulderAngleRad + inputs.elbowAngleRad)) * ELBOW_GRAVITY_GAIN_COEFFICIENT;
        Logger.getInstance().recordOutput("Arm/elbowGravityGain", elbowGravityGain);
        Logger.getInstance().recordOutput("Arm/ForearmFloorRelativeAngle", inputs.shoulderAngleRad + inputs.elbowAngleRad);

        // Convert our target pose (task space) to an arm state object (c-space) and update the IO object with new values.
        ArmState targetState = kinematics.inverse(targetPose);
        // Add ELBOW_BACKLASH_CONSTANT to compensate for backlash in the gearbox.
        targetState.elbowAngleRad += ELBOW_BACKLASH_CONSTANT;

        double shoulderVoltage = shoulderController.calculate(inputs.shoulderAngleRad, targetState.shoulderAngleRad);
        double elbowVoltage = elbowController.calculate(inputs.elbowAngleRad, targetState.elbowAngleRad) * (1 + elbowGravityGain);
        double wristVoltage = wristController.calculate(inputs.wristAngleRad, targetState.wristAngleRad);

        Logger.getInstance().recordOutput("Arm/ElbowSetpointAngle", elbowController.getSetpoint());
        Logger.getInstance().recordOutput("Arm/ElbowVoltage", elbowVoltage);
        
        io.setShoulderVoltage(applyJointLimits(shoulderVoltage, inputs.shoulderAngleRad, SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD));
        io.setElbowVoltage(applyJointLimits(elbowVoltage, inputs.elbowAngleRad, ELBOW_MIN_ANGLE_RAD, ELBOW_MAX_ANGLE_RAD));
        io.setWristVoltage(applyJointLimits(wristVoltage, inputs.wristAngleRad, WRIST_MIN_ANGLE_RAD, WRIST_MAX_ANGLE_RAD));   

        // Copy the target angles into an array we can pass to AdvantageKit to be logged.
        targetAnglesLogged[0] = targetState.shoulderAngleRad;
        targetAnglesLogged[1] = targetState.elbowAngleRad;
        targetAnglesLogged[2] = targetState.wristAngleRad;

        Logger.getInstance().recordOutput("Arm/Pose", targetPose);
        Logger.getInstance().recordOutput("Arm/TargetAngles", targetAnglesLogged);

        currentArmLigament.setAngle(Units.radiansToDegrees(inputs.shoulderAngleRad));
        currentForearmLigament.setAngle(Units.radiansToDegrees(inputs.elbowAngleRad));
        currentIntakeLigament.setAngle(Units.radiansToDegrees(inputs.wristAngleRad));

        targetPositionRoot.setPosition(targetPose.getX() + 2, targetPose.getY() + 1);
        targetPositionLigament.setAngle(targetPose.getRotation().getDegrees());

        targetArmLigament.setAngle(Units.radiansToDegrees(targetState.shoulderAngleRad));
        targetForearmLigament.setAngle(Units.radiansToDegrees(targetState.elbowAngleRad));
        targetIntakeLigament.setAngle(Units.radiansToDegrees(targetState.wristAngleRad));
        Logger.getInstance().recordOutput("Arm/Mechanism", mechanism);

        Pose2d currentPose = kinematics.forward(new ArmState(
            inputs.shoulderAngleRad,
            inputs.elbowAngleRad,
            inputs.wristAngleRad
        ));
        Logger.getInstance().recordOutput("Arm/CurrentPose", currentPose);
    }

    private double applyJointLimits(double voltage, double current, double min, double max) {
        if (current > max) {
            return Math.min(voltage, 0);
        }
        if (current < min) {
            return Math.max(voltage, 0);
        }
        return voltage;
    }

    public void setPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public Pose2d getPose() {
        return targetPose;
    }
}
