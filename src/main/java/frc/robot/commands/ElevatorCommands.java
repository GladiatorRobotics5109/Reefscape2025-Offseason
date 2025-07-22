package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.util.FieldConstants.Reef.ReefLevel;

import java.util.function.Supplier;

public class ElevatorCommands {
    public static Command toHeight(ElevatorSubsystem elevator, double heightMeters) {
        return Commands.runOnce(() -> elevator.setDesiredPositionMeters(heightMeters))
            .withName("ElevatorCommands::toHeight");
    }

    public static Command toReefLevel(ElevatorSubsystem elevator, Supplier<ReefLevel> level) {
        return Commands.runOnce(() -> elevator.setDesiredReefLevel(level.get()), elevator)
            .withName("ElevatorCommands::toReefLevel");
    }

    public static Command toReefLevel(ElevatorSubsystem elevator, ReefLevel level) {
        return toReefLevel(elevator, () -> level);
    }

    public static Command toMin(ElevatorSubsystem elevator) {
        return Commands.runOnce(() -> elevator.setDesiredPositionRad(ElevatorConstants.kMinPositionRad), elevator)
            .withName("ElevatorCommands::toMin");
    }
}
