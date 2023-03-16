package frc.robot.utility.config;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import lombok.Data;

@Data
public final class Falcon500Config {
    private int id;
    private String canBus = "rio";
    private boolean inverted = false;

    public TalonFX create() {
        WPI_TalonFX motor = new WPI_TalonFX(id, canBus);
        motor.setInverted(inverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);

        return motor;
    }
}
