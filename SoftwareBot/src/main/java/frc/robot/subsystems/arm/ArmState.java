package frc.robot.subsystems.arm;

public class ArmState {
    public double shoulderAngleRad;
    public double elbowAngleRad;
    public double wristAngleRad;

    public ArmState() {}

    public ArmState(double shoulderAngleRad, double elbowAngleRad, double wristAngleRad) {
       this.shoulderAngleRad = shoulderAngleRad;
       this.elbowAngleRad = elbowAngleRad;
       this.wristAngleRad = wristAngleRad; 
    }
}