// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Inake;
import frc.robot.commands.Score;
import frc.robot.commands.Stopintake;
import frc.robot.commands.lowerElevator;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  public void autonomousInit() {
    NamedCommands.registerCommand("ScoreHigh", new Score(2));
    NamedCommands.registerCommand("ScoreLow", new Score(1));
    NamedCommands.registerCommand("ScoreMid", new Score(0));
    NamedCommands.registerCommand("stopIntake", new Stopintake());
    NamedCommands.registerCommand("lowerElevator", new lowerElevator());
    NamedCommands.registerCommand("Pickup", new Inake());
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
