package frc.robot.utility.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.dataformat.yaml.YAMLGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utility.RobotIdentity;
import lombok.Data;

import java.io.IOException;
import java.nio.file.Paths;

@Data
public class RobotConfig {
    private ArmConfig arm = new ArmConfig();
    private DrivetrainConfig drivetrain = new DrivetrainConfig();
    private VisionConfig vision = new VisionConfig();

    public static RobotConfig loadConfig(RobotIdentity identity) throws IOException {
        var configPath = Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "config", identity.getConfigName());

        var om = new ObjectMapper(YAMLFactory.builder()
                .configure(YAMLGenerator.Feature.WRITE_DOC_START_MARKER, false)
                .configure(YAMLGenerator.Feature.MINIMIZE_QUOTES, true)
                .build());

        return om.readValue(configPath.toFile(), RobotConfig.class);
    }
}
