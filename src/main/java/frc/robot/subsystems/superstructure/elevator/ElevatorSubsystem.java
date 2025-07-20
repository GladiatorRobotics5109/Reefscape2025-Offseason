package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants.Reef.ReefLevel;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO m_io;
    private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

    @Getter
    @AutoLogOutput(key = ElevatorConstants.kLogPath + "/DesiredPositionRad")
    private double m_desiredPositionRad = 0.0;

    @Getter
    @AutoLogOutput(key = ElevatorConstants.kLogPath + "/DesiredReefLevel")
    private ReefLevel m_desiredReefLevel = ReefLevel.L1;

    @AutoLogOutput(key = ElevatorConstants.kLogPath + "/HasDesiredPosition")
    private boolean m_hasDesiredPosition = false;
    @AutoLogOutput(key = ElevatorConstants.kLogPath + "/HasDesiredReefLevel")
    private boolean m_hasDesiredReefLevel = false;

    @Getter
    @AutoLogOutput(key = ElevatorConstants.kLogPath + "/IsAtDesiredPosition")
    private boolean m_atDesiredPosition = false;

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
            ElevatorConstants.kV,
            ElevatorConstants.kA
        );
    }

    public void setVoltage(double volts) {
        m_hasDesiredPosition = false;
        m_hasDesiredReefLevel = false;
        m_io.setVoltage(volts);
    }

    public void stop() { setVoltage(0.0); }

    public void setDesiredPositionRad(double positionRad) {
        m_hasDesiredPosition = true;
        m_hasDesiredReefLevel = false;
        m_desiredPositionRad = positionRad;
    }

    public void setDesiredPositionMeters(double positionMeters) {
        setDesiredPositionRad(Conversions.elevatorMetersToRadians(positionMeters));
    }

    public void setDesiredReefLevel(ReefLevel level) {
        m_hasDesiredPosition = true;
        m_hasDesiredReefLevel = true;
        m_desiredReefLevel = level;
        m_desiredPositionRad = Conversions.elevatorMetersToRadians(
            level.getHeightMeters() + ElevatorConstants.kHeightOffsets.get(level)
        );
    }

    @AutoLogOutput
    public double getCurrentPositionRad() { return m_inputs.positionRad; }

    @AutoLogOutput
    public double getCurrentPositionMeters() { return Conversions.elevatorRadiansToMeters(getCurrentPositionRad()); }

    public boolean hasDesiredPosition() { return m_hasDesiredPosition; }

    public boolean hasDesiredReefLevel() { return m_hasDesiredReefLevel; }

    @Override
    public void periodic() {
        m_io.periodic();
        m_io.updateInputs(m_inputs);
        Logger.processInputs(ElevatorConstants.kIOLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }
        else if (m_hasDesiredPosition) {
            double pid = m_pid.calculate(
                m_inputs.positionRad,
                m_desiredPositionRad
            );
            State setpoint = m_pid.getSetpoint();
            Logger.recordOutput(ElevatorConstants.kLogPath + "/Profile/DesiredVelocity", setpoint.velocity);
            Logger.recordOutput(ElevatorConstants.kLogPath + "/Profile/DesiredPosition", setpoint.position);
            double ff = m_ff.calculate(setpoint.velocity);
            double desiredVoltage = pid + ff;

            Logger.recordOutput(ElevatorConstants.kLogPath + "/Profile/DesiredVoltage", desiredVoltage);
            m_io.setVoltage(desiredVoltage);
        }

        m_atDesiredPosition = MathUtil.isNear(
            m_desiredPositionRad,
            getCurrentPositionRad(),
            ElevatorConstants.kPositionToleranceRad
        );
    }
}
