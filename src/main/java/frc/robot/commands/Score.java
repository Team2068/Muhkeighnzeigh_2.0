// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.subWristem;

public class Score extends CommandBase {
  static final double highHeight = 1462;
  static final double midHeight = 1614;
  elevator Elevator;
  subWristem intake;
  double height;
  /** Creates a new Score. */
  public Score(int x, elevator Elevator, subWristem intake) {
    this.Elevator = Elevator;
    this.intake = intake;
    addRequirements(Elevator, intake);
    switch(x){
      case 0: height = 0; //lowest height
      break;
      case 1: height = midHeight; //mid height
      break;
      case 2: height = highHeight; //highest height 
      break;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator.setPosition(height);
    intake.setPosition(true);
    intake.setSpeed(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
