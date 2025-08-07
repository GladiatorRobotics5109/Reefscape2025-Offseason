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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
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

        public static final double kDriverControllerDeadzone = 0.1;
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
        public static final String kIOLogPath = "IO/DriveIO";
        public static final String kLogPath = "Subsystems/Drive";

        public static final double kAutoScoreMaxDistMeters = 1.75;
        public static final double kAutoScorePoseOffsetMeters = Conversions.inchesToMeters(15);

        public static final double kDefaultLinearSpeedMetersPerSec = 4.69;
        public static final double kSlowModeLinearSpeedMetersPerSec = 1.0;
        public static final double kDefaultAngularSpeedRadPerSec = Conversions.rotationsToRadians(1.5);
        public static final double kSlowModeAngularSpeedRadPerSec = Conversions.rotationsToRadians(0.75);

        public static final class DriveAtAngleConstants {
            public static final double kP = 5.0;
            public static final double kD = 0.4;
            public static final double kMaxVelocityRadPerSec = 8.0;
            public static final double kMaxAccelerationRadPerSec = 20.0;
        }

        public static final class DriveToPoseConstants {
            public static final double kTranslationP = 5.0;
            public static final double kTranslationI = 0.0;
            public static final double kTranslationD = 0.0;
            public static final double kTranslationMaxVel = 2.0;
            public static final double kTranslationAccel = 4.0;
            public static final double kTranslationToleranceMeters = Conversions.inchesToMeters(0.5);

            public static final double kRotationP = 5.0;
            public static final double kRotationI = 0.0;
            public static final double kRotationD = 0.0;
            public static final double kRotationMaxVel = 6.0;
            public static final double kRotationAccel = 12.0;
            public static final double kRotationToleranceRad = Conversions.degreesToRadians(0.5);
        }
    }

    public static final class ElevatorConstants {
        public static final String kIOLogPath = "IO/ElevatorIO";
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

        /** The height the base of the elevator is off the floor (for mechanism visualization) */
        public static final double kElevatorBaseHeightMeters = TunerConstants.FrontLeft.WheelRadius
            + Conversions.inchesToMeters(3.0 / 4.0);
        /** The amount the dispenser extends above the elevator (for mechanism visualization) */
        public static final double kDispenserHeightMeters = Conversions.inchesToMeters(21.5);
    }

    public static final class DispenserConstants {
        public static final String kIOLogPath = "IO/DispenserIO";
        public static final String kLogPath = "Subsystems/Dispenser";

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

    public static final class VisionConstants {
        // AprilTag layout
        public static AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2025ReefscapeWelded
        );

        public static final Transform3d kRobotToFrontCamera = new Transform3d(
            0.031 + Conversions.inchesToMeters(2.78),
            0.19,
            0.529 + Conversions.inchesToMeters(2),
            new Rotation3d(
                Conversions.degreesToRadians(-10),
                Conversions.degreesToRadians(33),
                Conversions.degreesToRadians(-19)
            )
        );
        public static final Transform3d kRobotToRearCamera = new Transform3d(
            -0.3302 - 0.13,
            -0.0889 - 0.03,
            0.51435,
            new Rotation3d(0.0, Conversions.degreesToRadians(10), Conversions.degreesToRadians(182))
        );
        // Basic filtering thresholds
        public static final double kMaxAmbiguity = 0.3;
        public static final double kMaxZError = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static final double kLinearStdDevBaseline = 0.02; // Meters
        public static final double kAngularStdDevBaseline = 0.06; // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static final double[] kCameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
        };

        // Multipliers to apply for MegaTag 2 observations
        public static final double kLinearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static final double kAngularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
    }
}
