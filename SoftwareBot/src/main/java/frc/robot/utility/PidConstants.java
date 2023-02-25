package frc.robot.utility;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@NoArgsConstructor
public class PidConstants {
    private double p;
    private double i;
    private double d;
}
