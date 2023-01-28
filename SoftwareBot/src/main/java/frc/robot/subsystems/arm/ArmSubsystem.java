package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class ArmSubsystem {
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(56);
    private static final double FOREARM_LENGTH_METERS = Units.inchesToMeters(48);
    private static final double HAND_LENGTH_METERS = Units.inchesToMeters(15);

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Pose2d targetPose;
    private ArmKinematics kinematics;




    public ArmSubsystem(ArmIO io){
        this.io = io;
        this.kinematics = new ArmKinematics();
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm/Inputs", inputs);

        ArmState state = kinematics.poseToArmState(targetPose);
    }

    public void setPose(Pose2d targetPose){
        this.targetPose = targetPose;
    }

    public Pose2d getPose(){
        return targetPose;
    }

    public void setShoulderAngle(double angle){

    }

    public void setElbowAngle(double angle){
        
    }

    public void setWristAngle(double angle){
        
    }
}
