package frc.robot.subsystems.superstructure.dispenser.sensors;

import org.littletonrobotics.junction.Logger;

public class DispenserSensors {
    private final DispenserSensorsIO m_io;
    private final DispenserSensorsIOInputsAutoLogged m_inputs;

    private final String m_logKey;

    public DispenserSensors(DispenserSensorsIO io, String logKey) {
        m_io = io;
        m_inputs = new DispenserSensorsIOInputsAutoLogged();

        m_logKey = logKey;
    }

    public boolean getSensor() { return m_inputs.sensorValue; }

    public boolean getLeadingSensor() { return m_inputs.leadingSensorValue; }

    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(m_logKey, m_inputs);
    }
}
