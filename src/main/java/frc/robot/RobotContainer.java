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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final DriveSubsystem m_drive;
    private ElevatorSubsystem m_elevator;

    // Controller
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

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

                break;
        }

        // Set up auto routines
        m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        m_autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(m_drive)
        );
        m_autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        m_autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        m_autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        // Configure the button bindings
        configureButtonBindings();
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
                () -> -m_driverController.getRightX()
            )
        );

        // Lock to 0° when A button is held
        m_driverController.a().whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> new Rotation2d()
            )
        );

        // Switch to X pattern when X button is pressed
        m_driverController.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

        // Reset gyro to 0° when B button is pressed
        m_driverController.b().onTrue(
            Commands.runOnce(
                () -> m_drive.setPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
                m_drive
            ).ignoringDisable(true)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //        return m_autoChooser.get();
        return Commands.runOnce(() -> m_elevator.setDesiredPositionRad(25), m_elevator);
    }
}
