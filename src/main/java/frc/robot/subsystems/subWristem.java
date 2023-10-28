// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class subWristem extends SubsystemBase {
  public CANSparkMax wristMotor;
  public CANSparkMax intakeMotor;
  
  public subWristem() {
    wristMotor = new CANSparkMax(69, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(420, MotorType.kBrushless);

    wristMotor.getEncoder().setPositionConversionFactor(123/456);

    wristMotor.getPIDController().setP(0);
    wristMotor.getPIDController().setI(0);
    wristMotor.getPIDController().setD(0);

    intakeMotor.getPIDController().setP(0);
    intakeMotor.getPIDController().setI(0);
    intakeMotor.getPIDController().setD(0);
  }

  public void setWristPosition(boolean isOut) {
    SmartDashboard.putBoolean("Is Intake Out", isOut);
    wristMotor.getPIDController().setReference(Math.toDegrees((isOut) ? 60 : 0), ControlType.kPosition);
  } 

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
