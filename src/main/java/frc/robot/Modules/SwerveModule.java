package frc.robot.Modules;

public interface SwerveModule {
    public void resetDrivePosition();
    public void resetSteerPosition();
    public double drivePosition();
    public double steerAngle();
    public void set(double driveVolts, double targetAngle);
}