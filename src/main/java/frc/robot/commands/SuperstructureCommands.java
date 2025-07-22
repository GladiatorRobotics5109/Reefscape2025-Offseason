package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.dispenser.DispenserSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.util.FieldConstants.Reef.ReefLevel;

public class SuperstructureCommands {
    public static Command intake(ElevatorSubsystem elevator, DispenserSubsystem dispenser) {
        return Commands.parallel(
            ElevatorCommands.toMin(elevator),
            DispenserCommands.intake(dispenser)
        ).withName("SuperstructureCommands::intake");
    }

    public static Command toReefLevel(ElevatorSubsystem elevator, DispenserSubsystem dispenser, ReefLevel level) {
        return Commands.sequence(
            Commands.waitUntil(() -> !dispenser.isIntaking()),
            ElevatorCommands.toReefLevel(elevator, level)
        ).withName("SuperstructureCommands::toReefLevel");
    }
}
