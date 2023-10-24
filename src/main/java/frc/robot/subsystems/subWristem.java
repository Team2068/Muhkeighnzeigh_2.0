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

public class subWristem extends SubsystemBase {
  public DutyCycleEncoder wristMotor;
  public CANSparkMax wristMotor;
  /** Creates a new wristSystem. */
  public subWristem() {
    wristMotor = new DutyCycleEncoder(0);
    wristMotor.setDutyCycleRange(0,1);
    wristMotor = new CANSparkMax(69, MotorType.kBrushless);
    wristMotor.reset();

    wristMotor.setPositionOffset(0);

    wristMotor.getPIDController().setP(0);
    wristMotor.getPIDController().setI(0);
    wristMotor.getPIDController().setD(0);
    wristMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    wristMotor.getEncoder().setPositionConversionFactor(360);
  }

  public void resetEncoder() {
    wristMotor.reset();
  }

  public double getPos() {
    return (wristMotor.getEncoder().getposition()) % 360;
  }

  public double encoderPos() {
    return wristMotor.getAbsolutePosition();
  }
  
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
