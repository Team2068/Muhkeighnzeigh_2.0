// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.     
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
  public CANSparkMax elevatorMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax elevatorMotor2 = new CANSparkMax(2, MotorType.kBrushless);

  /** Creates a new elevator. */
  public elevator() {
    elevatorMotor2.follow(elevatorMotor1, true);
  }

  public void move(double voltage){
    elevatorMotor1.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
