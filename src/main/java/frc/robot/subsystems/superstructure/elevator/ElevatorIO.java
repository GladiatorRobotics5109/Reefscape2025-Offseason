package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionRad;
    }

    void setVoltage(double volts);
}
