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

public class wristSystem extends SubsystemBase {
  public DutyCycleEncoder wristEncoder;
  public CANSparkMax wristMotor;
  /** Creates a new wristSystem. */
  public wristSystem() {
    wristEncoder = new DutyCycleEncoder(0);
    wristEncoder.setDutyCycleRange(0,1);
    wristMotor = new CANSparkMax(69, MotorType.kBrushless);
    wristEncoder.reset();

    wristEncoder.setPositionOffset(0);

    wristMotor.getPIDController().setP(0);
    wristMotor.getPIDController().setI(0);
    wristMotor.getPIDController().setD(0);
    wristMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    wristMotor.getEncoder().setPositionConversionFactor(360);
  }

  public void resetEncoder() {
    wristEncoder.reset();
  }

  public double getPos() {
    return (wristMotor.getEncoder().getposition()) % 360;
  }

  public double encoderPos() {
    return wristEncoder.getAbsolutePosition();
  }
  
  public void setVoltage(double voltage) {
    wristEncoder.setVoltage(voltage);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
