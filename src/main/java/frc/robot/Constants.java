package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class DriveConstants {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

        public static final int FRONT_LEFT_DRIVE_MOTOR = 9;
        public static final int FRONT_LEFT_TURN_MOTOR = 8;
        public static final int FRONT_LEFT_ENCODER = 17;
        public static double FRONT_LEFT_ENCODER_OFFSET;

        public static final int FRONT_RIGHT_DRIVE_MOTOR = 5;
        public static final int FRONT_RIGHT_TURN_MOTOR = 6;
        public static final int FRONT_RIGHT_ENCODER = 16;
        public static double FRONT_RIGHT_ENCODER_OFFSET;

        public static final int BACK_LEFT_DRIVE_MOTOR = 10;
        public static final int BACK_LEFT_TURN_MOTOR = 11;
        public static final int BACK_LEFT_ENCODER = 18;
        public static double BACK_LEFT_ENCODER_OFFSET;

        public static final int BACK_RIGHT_DRIVE_MOTOR = 4;
        public static final int BACK_RIGHT_TURN_MOTOR = 3;
        public static final int BACK_RIGHT_ENCODER = 15;
        public static double BACK_RIGHT_ENCODER_OFFSET;

        public static final int PIGEON_ID = 19;

        public static final void setOffsets() {
            FRONT_LEFT_ENCODER_OFFSET = -297;
            FRONT_RIGHT_ENCODER_OFFSET = -100;
            BACK_LEFT_ENCODER_OFFSET = -164;
            BACK_RIGHT_ENCODER_OFFSET = -53;
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 2.5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                1, 1);
    }

}
