package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;
import frc.robot.util.Conversions;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim m_sim = new ElevatorSim(
        DCMotor.getNEO(2),
        Constants.ElevatorConstants.kGearRatio,
        Conversions.lbsToKilograms(20),
        ElevatorConstants.kSprocketRadiusMeters,
        ElevatorConstants.kMinHeightMeters,
        ElevatorConstants.kMaxHeightMeters,
        true,
        0.0,
        0.0,
        0.0
    );

    private double m_inputVoltage;

    public ElevatorIOSim() {

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRad = Conversions.elevatorMetersToRadians(m_sim.getPositionMeters());
        inputs.velocityRadPerSec = Conversions.elevatorMetersToRadians(m_sim.getVelocityMetersPerSecond());

        inputs.motorTempCelsius = 0.0;
        inputs.motorAppliedVolts = m_sim.getInput(0);
        inputs.motorSupplyVoltage = 12;
        inputs.motorStatorCurrentAmps = m_sim.getCurrentDrawAmps();
        // V_supply * I_supply = V_applied * I_stator
        // I_supply = (V_applied * I_stator) / V_supply
        inputs.motorSupplyCurrentAmps = (inputs.motorAppliedVolts * inputs.motorStatorCurrentAmps)
            / inputs.motorSupplyVoltage;

        inputs.followerMotorTempCelsius = 0.0;
        inputs.followerMotorAppliedVolts = m_sim.getInput(0);
        inputs.followerMotorSupplyVoltage = 12;
        inputs.followerMotorStatorCurrentAmps = m_sim.getCurrentDrawAmps();
        inputs.followerMotorSupplyCurrentAmps = (inputs.followerMotorAppliedVolts
            * inputs.followerMotorStatorCurrentAmps) / inputs.followerMotorSupplyVoltage;
    }

    @Override
    public void setVoltage(double volts) {
        m_inputVoltage = volts;
        m_sim.setInputVoltage(volts);
    }

    @Override
    public void periodic() {
        m_sim.update(Robot.defaultPeriodSecs);
    }
}
