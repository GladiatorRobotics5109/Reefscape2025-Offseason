package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.dispenser.DispenserSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.Reef.ReefBranch;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class RobotCommands {
    public static Command autoScore(
        Supplier<ReefBranch> branchSupplier,
        DriveSubsystem drive,
        ElevatorSubsystem elevator,
        DispenserSubsystem dispenser
    ) {
        return Commands.parallel(
            Commands.runOnce(() -> {
                ReefBranch branch = branchSupplier.get();

                System.out.println("Auto scoring on branch: " + branch);
                Logger.recordOutput("AutoScore/DesiredBranch", FieldConstants.flipIfNecessary(branch.getPose()));
                Logger.recordOutput("AutoScore/DrivePose", Util.getDriveScorePose(branch));
            }),
            DriveCommands.driveToPose(
                drive,
                () -> FieldConstants.flipIfNecessary(Util.getDriveScorePose(branchSupplier.get()))
            )
            //            DriveCommands.driveToPose(drive, () -> {})
        );
    }
}
