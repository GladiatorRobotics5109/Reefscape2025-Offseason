package frc.robot.subsystems.superstructure.dispenser;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DispenserConstants;
import frc.robot.subsystems.superstructure.dispenser.sensors.DispenserSensors;
import frc.robot.subsystems.superstructure.dispenser.sensors.DispenserSensorsIO;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DispenserSubsystem extends SubsystemBase {
    private final DispenserIO m_io;
    private final DispenserIOInputsAutoLogged m_inputs;

    private final DispenserSensors m_sensors;

    @Getter
    private boolean m_intaking;

    public DispenserSubsystem(DispenserIO io, DispenserSensorsIO sensorsIO) {
        m_io = io;
        m_inputs = new DispenserIOInputsAutoLogged();

        m_sensors = new DispenserSensors(sensorsIO, DispenserConstants.kIOLogPath + "/Sensors");
    }

    @AutoLogOutput(key = DispenserConstants.kLogPath + "/HasCoral")
    public boolean hasCoral() { return m_sensors.getSensor(); }

    @AutoLogOutput(key = DispenserConstants.kLogPath + "/HasLeadingCoral")
    public boolean hasLeadingCoral() { return m_sensors.getLeadingSensor(); }

    public void setVoltage(double leftVolts, double rightVolts) { m_io.setVoltage(leftVolts, rightVolts); }

    public void setVoltage(double volts) { m_io.setVoltage(volts); }

    public void stop() {
        setVoltage(0.0);
        m_intaking = false;
    }

    public void runScore() { setVoltage(DispenserConstants.kScoreVoltage); m_intaking = false; }

    public void runIntake() {
        setVoltage(DispenserConstants.kIntakeVoltage);
        m_intaking = true;
    }

    public void runIntakeSlow() { setVoltage(DispenserConstants.kIntakeSlowVoltage); m_intaking = true; }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(DispenserConstants.kIOLogPath, m_inputs);

        m_sensors.periodic();
    }
}
