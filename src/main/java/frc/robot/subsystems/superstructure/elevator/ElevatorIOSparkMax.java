package frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax m_motor;
    private final SparkMax m_follower;

    private final RelativeEncoder m_encoder;

    public ElevatorIOSparkMax(int motorPort, int followerPort) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit((int)ElevatorConstants.kSupplyCurrentLimit);
        config.softLimit.forwardSoftLimit(Conversions.radiansToRotations(ElevatorConstants.kMaxPositionRad));
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(Conversions.radiansToRotations(ElevatorConstants.kMinPositionRad));
        config.softLimit.reverseSoftLimitEnabled(true);

        config.encoder.positionConversionFactor(1 / ElevatorConstants.kGearRatio);

        config.idleMode(IdleMode.kBrake);
        config.inverted(ElevatorConstants.kInvertMotor);

        m_motor = new SparkMax(motorPort, MotorType.kBrushless);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_follower = new SparkMax(followerPort, MotorType.kBrushless);
        m_follower.configure(
            config.follow(m_motor, true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRad = m_encoder.getPosition();
        inputs.velocityRadPerSec = m_encoder.getVelocity();

        inputs.motorTempCelsius = m_motor.getMotorTemperature();
        inputs.motorAppliedVolts = m_motor.getAppliedOutput();
        inputs.motorSupplyVoltage = m_motor.getBusVoltage();
        inputs.motorStatorCurrentAmps = m_motor.getOutputCurrent();
        // V_supply * I_supply = V_stator * I_stator
        // I_supply = (V_stator * I_stator) / V_supply
        inputs.motorSupplyCurrentAmps = (inputs.motorAppliedVolts * inputs.motorStatorCurrentAmps)
            / inputs.motorSupplyVoltage;

        inputs.followerMotorTempCelsius = m_follower.getMotorTemperature();
        inputs.followerMotorAppliedVolts = m_follower.getAppliedOutput();
        inputs.followerMotorSupplyVoltage = m_follower.getBusVoltage();
        inputs.followerMotorStatorCurrentAmps = m_follower.getOutputCurrent();
        inputs.followerMotorSupplyCurrentAmps = (inputs.followerMotorAppliedVolts
            * inputs.followerMotorStatorCurrentAmps) / inputs.followerMotorSupplyVoltage;
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }
}
