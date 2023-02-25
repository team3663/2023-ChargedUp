package frc.robot.utility.config;

import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmSubsystem;
import lombok.Data;

@Data
public class ArmConfig {
    private Falcon500Config[] shoulderMotors = {new Falcon500Config()};
    private Falcon500Config[] elbowMotors = new Falcon500Config[0];
    private Falcon500Config wristMotor = new Falcon500Config();

    public ArmSubsystem createSubsystem() {
        return new ArmSubsystem(new ArmIO() {
        });
    }
}
