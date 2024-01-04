// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// These classes do not exist on current robot code
// import frc.robot.Constants;
// import frc.robot.RobotState;
import frc.robot.subsystems.Limelight.LimelightConstants;
// import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {
    this(LimelightConstants.LedMode.DEFAULT, LimelightConstants.CamMode.VISION); // Error, Limelight constants needs to be defined  
  }
  public Limelight(int ledMode, int streamMode) { // Methods are undefined for type limelight
    setLedMode(ledMode);
    setStreamMode(streamMode);
  }

  // Structure for targeted data
  public class TargetData {
    public boolean hasTargets = false;
    public double horizontalOffset = 0;
    public double verticalOffset = 0;
    public double targetArea = 0;
    public double skew = 0;
    public double latency = 0;
    public double shortSideLength = 0;
    public double longSideLength = 0;
    public double horizontalSideLength = 0;
    public double verticalSideLength = 0;
  }
  private TargetData targetData = new TargetData();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    final String stream, cam;
    // Methods: getStreamMode, getCameraMode -- Undefined for type Limelight
    stream = (getStreamMode() == LimelightConstants.StreamMode.PIP_MAIN) ? "Main" : "Secondary";
    cam = (getCameraMode() == LimelightConstants.CamMode.VISION) ? "Vision" : "Driver";

    // getDistamce is undefined for type Limelight
    SmartDashboard.putString("Stream Mode", stream);
    SmartDashboard.putString("Camera Mode", cam);
    SmartDashboard.putNumber("Distance", getDistance());

    updateTargetData(table);
  }

  private void updateTargetData(NetworkTable table) {
    targetData.hasTargets = table.getEntry("tv").getBoolean(false);
    targetData.horizontalOffset = table.getEntry("tx").getDouble(0.0);
    targetData.verticalOffset = table.getEntry("ty").getDouble(0.0);
    targetData.targetArea = table.getEntry("ta").getDouble(0.0);
    targetData.skew = table.getEntry("ts").getDouble(0.0);
    targetData.latency = table.getEntry("tl").getDouble(0.0);
    targetData.shortSideLength = table.getEntry("tshort").getDouble(0.0);
    targetData.longSideLength = table.getEntry("tlong").getDouble(0.0);
    targetData.horizontalSideLength = table.getEntry("thor").getDouble(0.0);
    targetData.verticalSideLength = table.getEntry("tvert").getDouble(0.0);

  }
  public TargetData getTargetData() {
    return targetData;
  }

  // R^2 = 0.9687
  public double linearRPM() {
    double distance = getDistance();
    double factor = 2.86381 * distance;
    double rpm = factor + 2438;
    return rpm;
  }

  // R^2 = 0.9203
  public double curveRPM() {
    double distance = getDistance();
    double squared = distance * distance;
    return 0.0523285 * squared + 3101.62;
  }

  // R^2 = 0.9761
  public double logRPM() {
    double distance = getDistance();
    return Math.log(distance)/Math.log(1.00075) -4333.96;
  }

  // There is no shooter for Muhkeighnzeigh 2.0

  // This works only for objects that are above or below the robot
  // It's very inaccurate of objects that are same height as the robot
  public double getDistance() {
    TargetData targetData = getTargetData();
    double a2 = targetData.verticalOffset;
    double a1 = LimelightConstants.LIMELIGHT_ANGLE;
    double h1 = LimelightConstants.LIMELIGHT_HEIGHT;
    double h2 = 103 * 2.54; // UpperHub height in inches converted to cm

    double result = h2 - h1;
    double radians = Math.toRadians(a1 + a2);
    double distance = result / Math.tan(radians);

    return Math.abs(distance); // would return negative values if the angle was negative
  }

  public void toggleCameraMode(){
    switch (getCameraMode()) {
      case LimelightConstants.CamMode.DRIVER:
          setCameraMode(LimelightConstants.CamMode.VISION);
          break;
      default:
          setCameraMode(LimelightConstants.CamMode.DRIVER);
          break;
      }

  }

  public void toggleStreamMode(){
    switch (getStreamMode()) {
      case LimelightConstants.StreamMode.PIP_MAIN:
          setStreamMode(LimelightConstants.StreamMode.PIP_SECONDARY);
          break;
      case LimelightConstants.StreamMode.PIP_SECONDARY:
          setCameraMode(LimelightConstants.StreamMode.STANDARD);
      default:
          setStreamMode(LimelightConstants.StreamMode.PIP_MAIN);
          break;
      }
  }

  public void switchPipeline(){
    switch (getPipeline()) {
      case LimelightConstants.Pipelines.REFLECTIVE_TAPE:
          setPipeline(LimelightConstants.Pipelines.BRIGHT);
          break;
      case LimelightConstants.Pipelines.BRIGHT:
          setPipeline(LimelightConstants.Pipelines.REFLECTIVE_TAPE);
          break;
      }
  }

  public void setCameraMode(int newCameraMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(newCameraMode);
  }

  public void setLedMode(int newLedMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(newLedMode);
  }

  public void setPipeline(int newPipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(newPipeline);
  }

  public void setSnapshotMode(int newSnapshotMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(newSnapshotMode);
  }

  public void setStreamMode(int newStreamMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(newStreamMode);
  }

  public int getCameraMode() {
    return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getDouble(0.0);
  }

  public int getLedMode() {
    return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getDouble(0.0);
  }

  public int getPipeline() {
    return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0.0);
  }

  public int getSnapshotMode() {
    return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").getDouble(0.0);
  }

  public int getStreamMode() {
    return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").getDouble(0.0);
  }

  // Limelight Constants Class
  public final static class LimelightConstants {
    public final static class LedMode {
        public final static int DEFAULT = 0;
        public final static int FORCE_OFF = 1;
        public final static int FORCE_BLINK = 2;
        public final static int FORCE_ON = 3;
    }

    public final static class CamMode {
        public final static int VISION = 0;
        public final static int DRIVER = 1;
    }

    public final static class StreamMode {
        public final static int STANDARD = 0;
        public final static int PIP_MAIN = 1;
        public final static int PIP_SECONDARY = 2;
    }

    public final static class SnapshotMode {
        public final static int NO_SNAPSHOT = 0;
        public final static int TWO_SNAPSHOTS = 1;
    }

    public final static class Pipelines {
        public final static int REFLECTIVE_TAPE = 0;
        public final static int BRIGHT = 1;
    }

    public final static double LIMELIGHT_HEIGHT = 34.5 * 2.54; // Converting from inches to cm
    public final static double LIMELIGHT_ANGLE = 20;
  }
}
