// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Conversions;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static final class DriveConstants {
        public static final String kLogPath = "Subsystems/Drive";
    }

    public static final class ElevatorConstants {
        public static final String kLogPath = "ElevatorSubsystem";

        public static final double kP = 0.00;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.1;
        public static final double kV = 0.55;
        public static final double kA = 0.0;
        public static final double kG = 0.355;

        public static final double kGearRatio = 12;
        public static final double kSprocketRadiusMeters = Conversions.inchesToMeters(0.819);

        public static final double kMaxVelocityRadPerSec = 24.2;
        public static final double kMaxAccelerationRadPerSecPerSec = Conversions.elevatorMetersToRadians(1.5);

        public static final double kMinPositionRad = -0.03;
        public static final double kMaxPositionRad = 30.05;

        public static final double kMinHeightMeters = Conversions.elevatorRadiansToMeters(kMinPositionRad);
        public static final double kMaxHeightMeters = Conversions.elevatorRadiansToMeters(kMaxPositionRad);
    }
}
