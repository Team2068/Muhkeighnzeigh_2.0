// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSystem extends SubsystemBase {
  public CANSparkMax intakeMotor;
  /** Creates a new IntakeSystem. */
  public IntakeSystem() {
    intakeMotor = new CANSparkMax(69, MotorType.kBrushless);

    intakeEncoder.setPositionOffset(0);
    intakeMotor.getPIDController().setP(0);
    intakeMotor.getPIDController().setI(0);
    intakeMotor.getPIDController().setD(0);
  }

  public void setSpeed(double speed) {
    intakeEncoder.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
