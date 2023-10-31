// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Inake;
import frc.robot.commands.Score;
import frc.robot.commands.Stopintake;
import frc.robot.commands.defaultDrive;
import frc.robot.commands.lowerElevator;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  DriveSubsystem drive = new DriveSubsystem();

  public RobotContainer() {
    autonomousInit();
    DriveConstants.setOffsets();
    configureBindings();
  }

  public void autonomousInit() {
    // NamedCommands.registerCommand("ScoreHigh", new Score(2));
    // NamedCommands.registerCommand("ScoreLow", new Score(1));
    // NamedCommands.registerCommand("ScoreMid", new Score(0));
    // NamedCommands.registerCommand("stopIntake", new Stopintake());
    // NamedCommands.registerCommand("lowerElevator", new lowerElevator());
    // NamedCommands.registerCommand("Pickup", new Inake());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configureBindings() {
    drive.setDefaultCommand(new defaultDrive(drive, 
    () -> -modifyAxis(controller.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(controller.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(controller.getRightX())* DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

      controller.a().onTrue(new InstantCommand(drive::toggleFieldOriented));
      controller.b().onTrue(new InstantCommand(drive::toggleSlowMode));
      controller.x().onTrue(new InstantCommand(drive::resetOdometry));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) <= deadband)
        return 0.0;
    deadband *= (value > 0.0) ? 1 : -1;
    return (value + deadband) / (1.0 + deadband);
}

private static double modifyAxis(double value) {
    value = deadband(value, 0.05); // Deadband
    value = Math.copySign(value * value, value); // Square the axis
    return value;
}
}
