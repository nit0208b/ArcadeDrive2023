// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.util.DeferredCommand;
import frc.robot.commands.util.DeferredCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.util.CommandComposer;
import hlib.drive.Pose;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

	/** The PS4 controller the driver uses */
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);

	private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// -------------Driver Controls-------------
		// Driving
		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_driverController.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kLeftTrigger),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kRightTrigger)));

		// Fine Turning
		new JoystickButton(m_driverController, ControllerConstants.Button.kRightBumper)
				.whileTrue(new DefaultDriveCommand(() -> 0.0, () -> 0.0, () -> DriveConstants.kFineTurningSpeed));
		new JoystickButton(m_driverController, ControllerConstants.Button.kLeftBumper)
				.whileTrue(new DefaultDriveCommand(() -> 0.0, () -> DriveConstants.kFineTurningSpeed, () -> 0.0));
	}

	// TODO get auto command from auto chooser
	public Command getAutonomousCommand() {
		//return new BalancePIDCommand();
		return m_autoChooser.getSelected();
	}
}
