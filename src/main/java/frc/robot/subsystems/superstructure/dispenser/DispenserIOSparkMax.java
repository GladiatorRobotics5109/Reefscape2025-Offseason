package frc.robot.subsystems.superstructure.dispenser;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DispenserConstants;

public class DispenserIOSparkMax implements DispenserIO {
    private final SparkMax m_left;
    private final SparkMax m_right;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    public DispenserIOSparkMax(int leftPort, int rightPort) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit((int)DispenserConstants.kSupplyCurrentLimit);
        config.encoder.positionConversionFactor(1 / DispenserConstants.kGearRatio);

        config.idleMode(IdleMode.kBrake);
        config.inverted(DispenserConstants.kInvertMotor);

        m_left = new SparkMax(leftPort, MotorType.kBrushless);
        m_right = new SparkMax(rightPort, MotorType.kBrushless);

        m_leftEncoder = m_left.getEncoder();
        m_leftEncoder.setPosition(0.0);
        m_rightEncoder = m_right.getEncoder();
        m_rightEncoder.setPosition(0.0);
    }

    public void updateInputs(DispenserIOInputs inputs) {
        inputs.leftPositionRad = m_leftEncoder.getPosition();
        inputs.leftVelocityRadPerSec = m_leftEncoder.getVelocity();

        inputs.leftTempCelsius = m_left.getMotorTemperature();
        inputs.leftAppliedVolts = m_left.getAppliedOutput();
        inputs.leftSupplyVoltage = m_left.getBusVoltage();
        inputs.leftStatorCurrentAmps = m_left.getOutputCurrent();
        // V_supply * I_supply = V_stator * I_stator
        // I_supply = (V_stator * I_stator) / V_supply
        inputs.leftSupplyCurrentAmps = (inputs.leftAppliedVolts * inputs.leftStatorCurrentAmps)
            / inputs.leftSupplyVoltage;

        inputs.rightPositionRad = m_rightEncoder.getPosition();
        inputs.rightVelocityRadPerSec = m_rightEncoder.getVelocity();

        inputs.rightTempCelsius = m_right.getMotorTemperature();
        inputs.rightAppliedVolts = m_right.getAppliedOutput();
        inputs.rightSupplyVoltage = m_right.getBusVoltage();
        inputs.rightStatorCurrentAmps = m_right.getOutputCurrent();
        inputs.rightSupplyCurrentAmps = (inputs.rightAppliedVolts
            * inputs.rightStatorCurrentAmps) / inputs.rightSupplyVoltage;
    }

    public void setVoltage(double leftVolts, double rightVolts) {
        m_left.setVoltage(leftVolts);
        m_right.setVoltage(rightVolts);
    }
}
