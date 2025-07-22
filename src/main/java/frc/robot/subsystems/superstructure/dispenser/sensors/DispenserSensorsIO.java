package frc.robot.subsystems.superstructure.dispenser.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface DispenserSensorsIO {
    @AutoLog
    class DispenserSensorsIOInputs {
        public boolean sensorValue = false;
        public boolean leadingSensorValue = false;
    }

    default void updateInputs(DispenserSensorsIOInputs inputs) {}

    default boolean isReal() { return false; }
}
