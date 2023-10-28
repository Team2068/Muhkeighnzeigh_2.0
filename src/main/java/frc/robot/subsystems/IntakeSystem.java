package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSystem extends SubsystemBase {
  public CANSparkMax intakeMotor;
  /** Creates a new IntakeSystem. */
  public IntakeSystem() {
    intakeMotor = new CANSparkMax(69, MotorType.kBrushless);

    intakeMotor.getPIDController().setP(0);
    intakeMotor.getPIDController().setI(0);
    intakeMotor.getPIDController().setD(0);
  }
  
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
