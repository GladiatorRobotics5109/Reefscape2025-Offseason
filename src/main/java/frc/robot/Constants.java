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
import frc.robot.util.FieldConstants.Reef.ReefLevel;

import java.util.HashMap;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

    public static final class DriveTeamConstants {
        public static final int kDriverPort = 0;
        public static final int kOperatorPort = 1;

        public static final double kAutoScoreStartStopThreashold = 0.125;
        public static final double kAutoScoreSideSelectionThreshold = 0.125;
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static final class DriveConstants {
        public static final String kIOLogPath = "DriveIO";
        public static final String kLogPath = "Subsystems/Drive";

        public static final double kAutoScoreMaxDistMeters = 1.75;
        public static final double kAutoScorePoseOffsetMeters = Conversions.inchesToMeters(15);

        public static final class DriveToPoseConstants {
            public static final double kTranslationP = 5.0;
            public static final double kTranslationI = 0.0;
            public static final double kTranslationD = 0.0;
            public static final double kTranslationMaxVel = 2.0;
            public static final double kTranslationAccel = 1.0;
            public static final double kTranslationToleranceMeters = Conversions.inchesToMeters(0.5);

            public static final double kRotationP = 5.0;
            public static final double kRotationI = 0.0;
            public static final double kRotationD = 0.0;
            public static final double kRotationMaxVel = 6.0;
            public static final double kRotationAccel = 2.0;
            public static final double kRotationToleranceRad = Conversions.degreesToRadians(0.5);
        }
    }

    public static final class ElevatorConstants {
        public static final String kIOLogPath = "ElevatorIO";
        public static final String kLogPath = "Subsystems/Elevator";

        public static final double kP = 0.0;
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

        public static final double kSupplyCurrentLimit = 40.0;

        public static final int kMotorPort = 40;
        public static final boolean kInvertMotor = true;
        public static final int kFollowerPort = 41;

        public static final HashMap<ReefLevel, Double> kHeightOffsets = new HashMap<>();
        static {
            kHeightOffsets.put(ReefLevel.L1, 0.0);
            kHeightOffsets.put(ReefLevel.L2, 0.0);
            kHeightOffsets.put(ReefLevel.L3, 0.0);
            kHeightOffsets.put(ReefLevel.L4, 0.0);
        }

        public static final double kPositionToleranceRad = Conversions.elevatorMetersToRadians(
            Conversions.inchesToMeters(1)
        );
    }

    public static final class DispenserConstants {
        public static final String kIOLogPath = "DispenserIO";

        public static final double kGearRatio = 12;

        public static final boolean kInvertMotor = true;

        public static final int kLeftPort = 51;
        public static final int kRightPort = 50;

        public static final int kSensorPort = 1;
        public static final int kSensorLeadingPort = 2;

        public static final double kSupplyCurrentLimit = 40;

        public static final double kScoreVoltage = 5;
        public static final double kIntakeVoltage = 6;
        public static final double kIntakeSlowVoltage = 5;
        public static final double kIntakeSlowSlowVoltage = -1.75;
    }
}
