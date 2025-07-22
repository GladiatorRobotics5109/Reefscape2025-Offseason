package frc.robot.subsystems.superstructure.dispenser.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class DispenserSensorsIODigitalInput implements DispenserSensorsIO {
    private final DigitalInput m_sensor;
    private final DigitalInput m_leadingSensor;

    public DispenserSensorsIODigitalInput(int sensorChannel, int leadingSensorChannel) {
        m_sensor = new DigitalInput(sensorChannel);
        m_leadingSensor = new DigitalInput(leadingSensorChannel);
    }

    @Override
    public void updateInputs(DispenserSensorsIOInputs inputs) {
        inputs.sensorValue = m_sensor.get();
        inputs.leadingSensorValue = m_leadingSensor.get();
    }

    @Override
    public boolean isReal() { return true; }
}
