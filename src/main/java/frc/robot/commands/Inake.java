// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subWristem;

public class Inake extends CommandBase {
  subWristem intake = new subWristem();
  public Inake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPosition(true);
    intake.setSpeed(-1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
