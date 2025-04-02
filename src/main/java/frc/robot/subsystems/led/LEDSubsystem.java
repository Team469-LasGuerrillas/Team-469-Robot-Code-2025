package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private CANdle LED = new CANdle(0, "rio");
    private CANdleConfiguration ledConfig = new CANdleConfiguration();

    public LEDSubsystem() {
        ledConfig.brightnessScalar = 0.25;
        LED.configAllSettings(ledConfig);
    }

    @Override
    public void periodic() {
        LED.setLEDs(0, 255, 10);
    }
}
