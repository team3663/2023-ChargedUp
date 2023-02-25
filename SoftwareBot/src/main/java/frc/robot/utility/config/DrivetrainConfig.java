package frc.robot.utility.config;

import frc.robot.photonvision.IPhotonVision;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.utility.JsonUnits;
import frc.robot.utility.PidConstants;
import lombok.Data;

@Data
public final class DrivetrainConfig {
    @JsonUnits(JsonUnits.Unit.INCHES)
    private double trackwidth;
    @JsonUnits(JsonUnits.Unit.INCHES)
    private double wheelbase;
    @JsonUnits(JsonUnits.Unit.FEET_PER_SECOND)
    private double maxTranslationalVelocity;

    private PidConstants followerTranslationPid = new PidConstants(0.0, 0.0, 0.0);
    private PidConstants followerRotationPid = new PidConstants(0.0, 0.0, 0.0);

    private GyroConfig gyroscope;

    private SwerveModuleConfig swerveModule;

    private SwerveModuleConfig.HardwareConfig frontLeftModule;
    private SwerveModuleConfig.HardwareConfig frontRightModule;
    private SwerveModuleConfig.HardwareConfig backLeftModule;
    private SwerveModuleConfig.HardwareConfig backRightModule;

    public DrivetrainSubsystem createSubsystem(IPhotonVision photonVision) {
        var gyroIO = gyroscope == null ? new GyroIO() {
        } : gyroscope.createIO();

        var frontLeftModuleIO = frontLeftModule == null ? new SwerveModuleIO() {
        } : swerveModule.createIO(frontLeftModule);
        var frontRightModuleIO = frontRightModule == null ? new SwerveModuleIO() {
        } : swerveModule.createIO(frontRightModule);
        var backLeftModuleIO = backLeftModule == null ? new SwerveModuleIO() {
        } : swerveModule.createIO(backLeftModule);
        var backRightModuleIO = backRightModule == null ? new SwerveModuleIO() {
        } : swerveModule.createIO(backRightModule);

        return new DrivetrainSubsystem(this, gyroIO, frontLeftModuleIO, frontRightModuleIO, backLeftModuleIO, backRightModuleIO, photonVision);
    }
}
