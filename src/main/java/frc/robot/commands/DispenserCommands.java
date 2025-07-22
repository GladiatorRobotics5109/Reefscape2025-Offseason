package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.dispenser.DispenserSubsystem;

public class DispenserCommands {
    public static Command score(DispenserSubsystem dispenser) {
        return Commands.sequence(
            Commands.runOnce(dispenser::runScore, dispenser),
            Commands.waitUntil(() -> !dispenser.hasCoral()),
            Commands.waitSeconds(0.2),
            Commands.runOnce(dispenser::stop, dispenser)
        ).withName("DispenserCommands::score");
    }

    public static Command intake(DispenserSubsystem dispenser) {
        return Commands.sequence(
            Commands.runOnce(dispenser::runIntake, dispenser),
            Commands.waitUntil(dispenser::hasLeadingCoral),
            Commands.runOnce(dispenser::runIntakeSlow, dispenser),
            Commands.waitUntil(() -> !dispenser.hasLeadingCoral()),
            Commands.runOnce(dispenser::stop, dispenser)
        );
    }
}
