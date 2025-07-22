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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.superstructure.dispenser.DispenserIO;
import frc.robot.subsystems.superstructure.dispenser.DispenserIOSparkMax;
import frc.robot.subsystems.superstructure.dispenser.DispenserSubsystem;
import frc.robot.subsystems.superstructure.dispenser.sensors.DispenserSensorsIO;
import frc.robot.subsystems.superstructure.dispenser.sensors.DispenserSensorsIODigitalInput;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.Reef;
import frc.robot.util.FieldConstants.Reef.ReefBranch;
import frc.robot.util.FieldConstants.Reef.ReefFaceSide;
import frc.robot.util.FieldConstants.Reef.ReefLevel;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final DriveSubsystem m_drive;
    private final ElevatorSubsystem m_elevator;
    private final DispenserSubsystem m_dispenser;

    // Controller
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(DriveTeamConstants.kDriverPort);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    private boolean m_hasAutoScoreBranch;
    private ReefBranch m_autoScoreBranch;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive = new DriveSubsystem(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight)
                );

                m_elevator = new ElevatorSubsystem(
                    new ElevatorIOSparkMax(ElevatorConstants.kMotorPort, ElevatorConstants.kFollowerPort)
                );

                m_dispenser = new DispenserSubsystem(
                    new DispenserIOSparkMax(DispenserConstants.kLeftPort, DispenserConstants.kRightPort),
                    new DispenserSensorsIODigitalInput(
                        DispenserConstants.kSensorPort,
                        DispenserConstants.kSensorLeadingPort
                    )
                );

                break;
            case SIM:
                DriverStation.silenceJoystickConnectionWarning(true);

                // Sim robot, instantiate physics sim IO implementations
                m_drive = new DriveSubsystem(
                    new GyroIO() {},
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight)
                );

                m_elevator = new ElevatorSubsystem(new ElevatorIOSim());

                m_dispenser = new DispenserSubsystem(new DispenserIO() {}, new DispenserSensorsIO() {});

                break;
            default:
                // Replayed robot, disable IO implementations
                m_drive = new DriveSubsystem(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                );

                m_elevator = new ElevatorSubsystem(new ElevatorIO() {});

                m_dispenser = new DispenserSubsystem(new DispenserIO() {}, new DispenserSensorsIO() {});

                break;
        }

        // Set up auto routines
        m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        m_autoChooser.addOption(
            "Test_DriveWheelRadiusCharacterization",
            DriveCommands.wheelRadiusCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "Test_DriveSimpleFFCharacterization",
            DriveCommands.feedforwardCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "Test_DriveSysId(Quasistatic Forward)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "Test_DriveSysId(Quasistatic Reverse)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        m_autoChooser.addOption(
            "Test_DriveSysId(Dynamic Forward)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "Test_DriveSysId(Dynamic Reverse)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        // Configure the button bindings
        configureButtonBindings();

        for (ReefBranch val : ReefBranch.values()) {
            Logger.recordOutput("BranchPoses/" + val.name(), val.getPose());
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                m_driverController::getR2Axis
            )
        );

        m_driverController.R1()
            .onTrue(Commands.runOnce(m_dispenser::runScore, m_dispenser))
            .onFalse(Commands.runOnce(m_dispenser::stop, m_dispenser));

        m_driverController.L1()
            .onTrue(SuperstructureCommands.intake(m_elevator, m_dispenser));

        m_driverController.cross().onTrue(ElevatorCommands.toMin(m_elevator));
        m_driverController.circle().onTrue(ElevatorCommands.toReefLevel(m_elevator, ReefLevel.L2));
        m_driverController.triangle().onTrue(ElevatorCommands.toReefLevel(m_elevator, ReefLevel.L3));
        m_driverController.square().onTrue(ElevatorCommands.toReefLevel(m_elevator, ReefLevel.L4));

        m_driverController.L2()
            .and(() -> Math.abs(m_driverController.getL2Axis()) >= DriveTeamConstants.kAutoScoreStartStopThreashold)
            .and(this::isReadyToAutoScore)
            .whileTrue(DriveCommands.joystickDrive(m_drive, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0))
            .and(() -> Math.abs(m_driverController.getLeftX()) >= DriveTeamConstants.kAutoScoreSideSelectionThreshold)
            .onTrue(Commands.runOnce(() -> {
                Optional<ReefBranch> branch = Util.decideAutoScoreBranch(
                    ReefFaceSide.fromDouble(m_driverController.getLeftX()),
                    m_drive.getPose(),
                    m_elevator.getDesiredReefLevel()
                );
                if (branch.isPresent()) {
                    m_autoScoreBranch = branch.get();
                    m_hasAutoScoreBranch = true;
                }
                else {
                    DriverStation.reportWarning("Failed to decide auto score branch!", true);
                    m_hasAutoScoreBranch = false;
                }
            }));
        m_driverController.L2()
            .and(() -> Math.abs(m_driverController.getL2Axis()) >= DriveTeamConstants.kAutoScoreStartStopThreashold)
            .and(this::isReadyToAutoScore)
            .and(() -> m_hasAutoScoreBranch)
            .onFalse(
                RobotCommands.autoScore(() -> m_autoScoreBranch, m_drive, m_elevator, m_dispenser)
                    .until(() -> m_driverController.getL2Axis() >= DriveTeamConstants.kAutoScoreStartStopThreashold)
                    .finallyDo(() -> m_hasAutoScoreBranch = false)
            );

        // m_driverController.options()
        //     .and(this::isReadyToAutoScore)
        //     .and(() -> Math.abs(m_driverController.getLeftX()) >= DriveTeamConstants.kAutoScoreSideSelectionThreshold)
        //     .onTrue(
        //         RobotCommands.autoScore(
        //             () -> Util.decideAutoScoreBranch(
        //                 ReefFaceSide.fromDouble(m_driverController.getLeftX()),
        //                 m_drive.getPose(),
        //                 ReefLevel.L4
        //             ),
        //             m_drive,
        //             m_elevator,
        //             m_dispenser
        //         )
        //     );

        // // Lock to 0° when A button is held
        // m_driverController.a().whileTrue(
        //     DriveCommands.joystickDriveAtAngle(
        //         m_drive,
        //         () -> -m_driverController.getLeftY(),
        //         () -> -m_driverController.getLeftX(),
        //         () -> new Rotation2d()
        //     )
        // );

        // // Switch to X pattern when X button is pressed
        // m_driverController.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

        // // Reset gyro to 0° when B button is pressed
        // m_driverController.b().onTrue(
        //     Commands.runOnce(
        //         () -> m_drive.setPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
        //         m_drive
        //     ).ignoringDisable(true)
        // );
    }

    public boolean isReadyToAutoScore() {
        return m_elevator.hasDesiredReefLevel()
            && (Constants.kCurrentMode == Mode.SIM || m_dispenser.hasCoral()) // If sim, assume we have coral
            && m_drive.getPose().getTranslation().getDistance(FieldConstants.flipIfNecessary(Reef.kReefPose))
                <= DriveConstants.kAutoScoreMaxDistMeters;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.get();
    }
}
