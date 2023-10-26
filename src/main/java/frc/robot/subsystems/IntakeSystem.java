// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSystem extends SubsystemBase {
  public DutyCycleEncoder intakeEncoder;
  public CANSparkMax intakeMotor;
  /** Creates a new IntakeSystem. */
  public IntakeSystem() {
    intakeEncoder = new DutyCycleEncoder(0);
    intakeEncoder.setDutyCycleRange(0,1);
    intakeMotor = new CANSparkMax(69, MotorType.kBrushless);
    intakeEncoder.reset();

    intakeEncoder.setPositionOffset(0);

    intakeMotor.getPIDController().setP(0);
    intakeMotor.getPIDController().setI(0);
    intakeMotor.getPIDController().setD(0);
    intakeMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    intakeMotor.getEncoder().setPositionConversionFactor(360);
  }

  public void resetEncoder() {
    intakeEncoder.reset();
  }

  public double getPos() {
    return (intakeMotor.getEncoder().getPosition()) % 360;
  }

  public double encoderPos() {
    return intakeEncoder.getAbsolutePosition();
  }
  
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
