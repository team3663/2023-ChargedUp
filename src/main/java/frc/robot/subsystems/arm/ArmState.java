package frc.robot.subsystems.arm;

public class ArmState {
    public double shoulderAngleRad;
    public double elbowAngleRad;
    public double wristAngleRad;
    public boolean valid;

    public ArmState() {}

    public ArmState(double shoulderAngleRad, double elbowAngleRad, double wristAngleRad, boolean valid) {
       this.shoulderAngleRad = shoulderAngleRad;
       this.elbowAngleRad = elbowAngleRad;
       this.wristAngleRad = wristAngleRad; 
       this.valid = valid;
    }

    public ArmState(double shoulderAngleRad, double elbowAngleRad, double wristAngleRad) {
        this(shoulderAngleRad, elbowAngleRad,wristAngleRad, true);
     }
}