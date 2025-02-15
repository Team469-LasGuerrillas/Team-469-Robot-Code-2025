package frc.lib.interfaces.motor;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class CancoderConfigs {
    public int canId; 
    public String canBus;
    public final CANcoderConfiguration ccConfig = new CANcoderConfiguration();
}
