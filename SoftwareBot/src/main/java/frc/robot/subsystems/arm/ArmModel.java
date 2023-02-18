package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmModel {
    private final ArmLinkage armLinkage;
    private final Translation2d armCgOffset;
    private final double armMass;
    private final double armCgMoi;

    private final ArmLinkage forearmLinkage;
    private final Translation2d forearmCgOffset;
    private final double forearmMass;
    private final double forearmCgMoi;

    private final ArmLinkage handLinkage;
    private final Translation2d handCgOffset;
    private final double handMass;
    private final double handCgMoi;

    public ArmModel(ArmLinkage armLinkage, Translation2d armCgOffset, double armMass, double armCgMoi,
                    ArmLinkage forearmLinkage, Translation2d forearmCgOffset, double forearmMass, double forearmCgMoi,
                    ArmLinkage handLinkage, Translation2d handCgOffset, double handMass, double handCgMoi) {
        this.armLinkage = armLinkage;
        this.armCgOffset = armCgOffset;
        this.armMass = armMass;
        this.armCgMoi = armCgMoi;
        this.forearmLinkage = forearmLinkage;
        this.forearmCgOffset = forearmCgOffset;
        this.forearmMass = forearmMass;
        this.forearmCgMoi = forearmCgMoi;
        this.handLinkage = handLinkage;
        this.handCgOffset = handCgOffset;
        this.handMass = handMass;
        this.handCgMoi = handCgMoi;
    }
}
