package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.FieldConstants.Reef.ReefBranch;
import frc.robot.util.FieldConstants.Reef.ReefFace;
import frc.robot.util.FieldConstants.Reef.ReefFaceSide;
import frc.robot.util.FieldConstants.Reef.ReefLevel;
import lombok.experimental.UtilityClass;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;

@UtilityClass
public class Util {
    /**
     * Decides which branch we should score on
     *
     * @param side Left or right side of reef face
     * @param currentPose Current pose of the robot
     * @param level Desired reef level
     *
     * @return Branch to score on
     */
    public Optional<ReefBranch> decideAutoScoreBranch(ReefFaceSide side, Pose2d currentPose, ReefLevel level) {
        // First 2 faces should be the faces with the closest base
        ReefFace[] desiredFaces = Arrays.stream(ReefFace.values()).sorted(
            Comparator.comparingDouble(
                face -> FieldConstants.flipIfNecessary(face.getTranslation()).getDistance(currentPose.getTranslation())
            )
        ).toArray(ReefFace[]::new);

        Optional<ReefFace> desiredFaceOpt = Optional.empty();
        for (int i = 0; i < 2; i++) {
            if (desiredFaces[i].getSide() == side) {
                desiredFaceOpt = Optional.of(desiredFaces[i]);
                break;
            }
        }
        if (desiredFaceOpt.isEmpty()) {
            DriverStation.reportWarning("Could not decide auto score branch!", true);
            return Optional.empty();
        }
        ReefFace desiredFace = desiredFaceOpt.get();

        return Arrays.stream(ReefBranch.values())
            .filter(branch -> branch.getLevel() == level && branch.getFace() == desiredFace)
            .findFirst();
    }

    /**
     * Returns the desired robot pose to score at the provided branch
     *
     * @param branch Desired branch to score on
     *
     * @return Pose associated with branch
     */
    public Pose2d getDriveScorePose(ReefBranch branch) {
        Rotation2d angle = branch.getPose().toPose2d().getRotation();

        return new Pose2d(
            branch.getPose().toPose2d().getTranslation().plus(
                new Translation2d(DriveConstants.kAutoScorePoseOffsetMeters, angle)
            ),
            angle.plus(Rotation2d.k180deg)
        );
    }
}
