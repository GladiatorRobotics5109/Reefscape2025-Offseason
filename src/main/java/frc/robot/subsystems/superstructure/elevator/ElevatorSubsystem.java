package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.util.Conversions;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO m_io;
    private final ElevatorIOInputs m_inputs = new ElevatorIOInputsAutoLogged();

    @Getter
    @AutoLogOutput(key = ElevatorConstants.kLogPath + "/DesiredPositionRad")
    private double m_desiredPositionRad = 0.0;

    @Getter
    @AutoLogOutput(key = ElevatorConstants.kLogPath + "/HasDesiredPosition")
    private boolean m_hasDesiredPosition = false;

    private final ProfiledPIDController m_pid;
    private final ElevatorFeedforward m_ff;

    public ElevatorSubsystem(ElevatorIO io) {
        m_io = io;

        m_pid = new ProfiledPIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxVelocityRadPerSec,
                ElevatorConstants.kMaxAccelerationRadPerSecPerSec
            )
        );
        m_ff = new ElevatorFeedforward(
            ElevatorConstants.kS,
            ElevatorConstants.kG,
            ElevatorConstants.kS,
            ElevatorConstants.kA
        );
    }

    public void setVoltage(double volts) {
        m_hasDesiredPosition = false;
        m_io.setVoltage(volts);
    }

    public void stop() { setVoltage(0.0); }

    public void setDesiredPositionRad(double positionRad) {
        m_hasDesiredPosition = true;
        m_desiredPositionRad = positionRad;
    }

    public void setDesiredPositionMeters(double positionMeters) {
        setDesiredPositionRad(Conversions.elevatorPositionMetersToRadians(positionMeters));
    }

    @AutoLogOutput
    public double getCurrentPositionRad() { return m_inputs.positionRad; }

    @AutoLogOutput
    public double getCurrentPositionMeters() {
        return Conversions.elevatorPositionRadiansToMeters(getCurrentPositionRad());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
        }
        else if (m_hasDesiredPosition) {
            double pid = m_pid.calculate(
                m_inputs.positionRad,
                m_desiredPositionRad
            );
            double ff = m_ff.calculate(m_pid.getSetpoint().velocity);
            double desiredVoltage = pid + ff;

            Logger.recordOutput(ElevatorConstants.kLogPath + "/DesiredVoltage", desiredVoltage);
            m_io.setVoltage(desiredVoltage);
        }
    }
}
