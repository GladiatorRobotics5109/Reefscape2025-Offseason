package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.Getter;
import lombok.experimental.UtilityClass;

import java.util.Optional;

@UtilityClass
public class FieldConstants {
    @UtilityClass
    public class Reef {
        public static final double kL1HeightMeters = Conversions.feetToMeters(1) + Conversions.inchesToMeters(6);
        public static final double kL2HeightMeters = Conversions.feetToMeters(2)
            + Conversions.inchesToMeters(7 + (7 / 8));
        public static final double kL3HeightMeters = Conversions.feetToMeters(3)
            + Conversions.inchesToMeters(11 + (5 / 8));
        public static final double kL4HeightMeters = Conversions.feetToMeters(6);

        public static final Translation2d kReefPose = new Translation2d(
            Conversions.inchesToMeters(176.75),
            Conversions.inchesToMeters(158.5)
        );
        public static final double kReefDiameterMeters = Conversions.inchesToMeters(65.5);
        public static final double kReefRadiusMeters = kReefDiameterMeters / 2;

        public static final double kReefBranchDistMeters = Conversions.inchesToMeters(13);

        public enum ReefLevel {
            L1(1, kL1HeightMeters),
            L2(2, kL2HeightMeters),
            L3(3, kL3HeightMeters),
            L4(4, kL4HeightMeters);

            @Getter
            private final double m_heightMeters;
            @Getter
            private final int m_level;
            @Getter
            private final Rotation2d m_angle;

            ReefLevel(int level, double heightMeters) {
                m_level = level;
                m_heightMeters = heightMeters;

                m_angle = Rotation2d.fromDegrees(switch (level) {
                    case 2, 3 -> 35;
                    case 4 -> 90;
                    default -> 0;
                });
            }

            public static ReefLevel fromLevel(int i) {
                return switch (i) {
                    case 1 -> ReefLevel.L1;
                    case 2 -> ReefLevel.L2;
                    case 3 -> ReefLevel.L3;
                    default -> ReefLevel.L4;
                };
            }
        }

        public enum ReefFaceSide {
            Left,
            Right;

            public static ReefFaceSide fromDouble(double d) {
                if (d >= 0.0f) return ReefFaceSide.Right;

                return ReefFaceSide.Left;
            }
        }

        public enum ReefFace {
            A(Rotation2d.kPi, ReefFaceSide.Left),
            B(Rotation2d.kPi, ReefFaceSide.Right),
            C(Rotation2d.fromDegrees(-120), ReefFaceSide.Left),
            D(Rotation2d.fromDegrees(-120), ReefFaceSide.Right),
            E(Rotation2d.fromDegrees(-60), ReefFaceSide.Right),
            F(Rotation2d.fromDegrees(-60), ReefFaceSide.Left),
            G(Rotation2d.kZero, ReefFaceSide.Right),
            H(Rotation2d.kZero, ReefFaceSide.Left),
            I(Rotation2d.fromDegrees(60), ReefFaceSide.Right),
            J(Rotation2d.fromDegrees(60), ReefFaceSide.Left),
            K(Rotation2d.fromDegrees(120), ReefFaceSide.Left),
            L(Rotation2d.fromDegrees(120), ReefFaceSide.Right);

            /** The direction that the face faces relative to the center of the blue reef */
            @Getter
            private final Rotation2d m_angle;
            /** The translation of the base */
            @Getter
            private final Translation2d m_baseTranslation;
            @Getter
            private final Translation2d m_translation;
            @Getter
            private final Pose2d m_pose;
            /** The side this face relative to the driver station */
            @Getter
            private final ReefFaceSide m_side;

            ReefFace(Rotation2d angle, ReefFaceSide side) {
                m_angle = angle;
                m_side = side;
                m_baseTranslation = Reef.kReefPose.plus(new Translation2d(Reef.kReefRadiusMeters, m_angle));
                m_translation = m_baseTranslation.plus(
                    new Translation2d(
                        Reef.kReefBranchDistMeters / 2,
                        m_angle.plus(side == ReefFaceSide.Right ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg)
                    )
                );
                m_pose = new Pose2d(m_translation, m_angle);
            }
        }

        public enum ReefBranch {
            L2A(ReefLevel.L2, ReefFace.A),
            L2B(ReefLevel.L2, ReefFace.B),
            L2C(ReefLevel.L2, ReefFace.C),
            L2D(ReefLevel.L2, ReefFace.D),
            L2E(ReefLevel.L2, ReefFace.E),
            L2F(ReefLevel.L2, ReefFace.F),
            L2G(ReefLevel.L2, ReefFace.G),
            L2H(ReefLevel.L2, ReefFace.H),
            L2I(ReefLevel.L2, ReefFace.I),
            L2J(ReefLevel.L2, ReefFace.J),
            L2K(ReefLevel.L2, ReefFace.K),
            L2L(ReefLevel.L2, ReefFace.L),

            L3A(ReefLevel.L3, ReefFace.A),
            L3B(ReefLevel.L3, ReefFace.B),
            L3C(ReefLevel.L3, ReefFace.C),
            L3D(ReefLevel.L3, ReefFace.D),
            L3E(ReefLevel.L3, ReefFace.E),
            L3F(ReefLevel.L3, ReefFace.F),
            L3G(ReefLevel.L3, ReefFace.G),
            L3H(ReefLevel.L3, ReefFace.H),
            L3I(ReefLevel.L3, ReefFace.I),
            L3J(ReefLevel.L3, ReefFace.J),
            L3K(ReefLevel.L3, ReefFace.K),
            L3L(ReefLevel.L3, ReefFace.L),

            L4A(ReefLevel.L4, ReefFace.A),
            L4B(ReefLevel.L4, ReefFace.B),
            L4C(ReefLevel.L4, ReefFace.C),
            L4D(ReefLevel.L4, ReefFace.D),
            L4E(ReefLevel.L4, ReefFace.E),
            L4F(ReefLevel.L4, ReefFace.F),
            L4G(ReefLevel.L4, ReefFace.G),
            L4H(ReefLevel.L4, ReefFace.H),
            L4I(ReefLevel.L4, ReefFace.I),
            L4J(ReefLevel.L4, ReefFace.J),
            L4K(ReefLevel.L4, ReefFace.K),
            L4L(ReefLevel.L4, ReefFace.L);

            @Getter
            private final ReefLevel m_level;
            @Getter
            private final ReefFace m_face;
            @Getter
            private final Pose3d m_pose;

            ReefBranch(ReefLevel level, ReefFace face) {
                m_level = level;
                m_face = face;
                Rotation2d faceAngle = face.getAngle();

                Pose2d facePose = face.getPose();
                m_pose = new Pose3d(
                    facePose.getX(),
                    facePose.getY(),
                    level.getHeightMeters(),
                    new Rotation3d(0.0, -level.getAngle().getRadians(), faceAngle.getRadians())
                );

                // m_pose = new Pose3d(
                //     kReefPose.getX()
                //         + kReefRadiusMeters * faceAngle.getCos()
                //         + kReefBranchDistMeters / 2 * faceAngle.plus(Rotation2d.kCCW_90deg).getCos(),
                //     kReefPose.getY()
                //         + kReefRadiusMeters * faceAngle.getSin()
                //         + kReefBranchDistMeters / 2 * faceAngle.plus(Rotation2d.kCCW_90deg).getSin(),
                //     level.getHeightMeters(),
                //     new Rotation3d(0.0, -level.getAngle().getRadians(), faceAngle.getRadians())
                // );
            }
        }
    }

    private boolean currentAllianceIs(Alliance alliance) {
        Optional<Alliance> current = DriverStation.getAlliance();

        return current.isPresent() && current.get() == alliance;
    }

    public boolean shouldFlip() {
        return currentAllianceIs(Alliance.Red);
    }

    public Pose2d flipIfNecessary(Pose2d pose) {
        return shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public Pose3d flipIfNecessary(Pose3d pose) {
        return shouldFlip()
            ? new Pose3d(
                FlippingUtil.fieldSizeX - pose.getX(),
                pose.getY(),
                pose.getZ(),
                flipIfNecessary(pose.getRotation())
            )
            : pose;
    }

    public Translation2d flipIfNecessary(Translation2d translation) {
        return shouldFlip() ? FlippingUtil.flipFieldPosition(translation) : translation;
    }

    public Rotation2d flipIfNecessary(Rotation2d rotation) {
        return shouldFlip() ? rotation.plus(Rotation2d.kPi) : rotation;
    }

    public Rotation3d flipIfNecessary(Rotation3d rotation) {
        return shouldFlip() ? rotation.plus(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
    }
}
