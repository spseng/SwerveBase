// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SingleSwerveTest;
import frc.robot.subsystems.SwerveModule;

public class RobotContainer {
	public static final CommandXboxController driver = new CommandXboxController(0);
	public static final CommandXboxController operator = new CommandXboxController(1);

	private SwerveModule _module;


	public RobotContainer() {
		_module = new SwerveModule(RobotMap.CAN.FL_STEER, RobotMap.CAN.FL_DRIVE, RobotMap.CAN.FL_ENCODER, Constants.Drivetrain.FL_CONFIG);

		_module.setDefaultCommand(new SingleSwerveTest(_module));

		configureBindings();
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
