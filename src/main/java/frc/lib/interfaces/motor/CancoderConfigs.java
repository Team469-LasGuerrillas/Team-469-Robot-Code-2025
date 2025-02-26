package frc.lib.interfaces.motor;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class CancoderConfigs {
    public int canId; 
    public String canBus;
    public CANcoderConfiguration ccConfig = new CANcoderConfiguration();

    public CancoderConfigs withCanId(int canId) {
        this.canId = canId;
        return this;
    }

    public CancoderConfigs withCanBus(String canBus) {
        this.canBus = canBus;
        return this;
    }

    public CancoderConfigs withCcConfig(CANcoderConfiguration ccConfig) {
        this.ccConfig = ccConfig;
        return this;
    }
}
