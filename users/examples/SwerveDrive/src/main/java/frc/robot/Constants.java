package frc.robot;

public class Constants {

    public static int FRONT_LEFT_TURNING_CAN_ID     = 20;
    public static int FRONT_LEFT_DRIVING_CAN_ID     = 10;
    public static int FRONT_LEFT_CANCODER_ID        = 30;

    public static int FRONT_RIGHT_TURNING_CAN_ID    = 22;
    public static int FRONT_RIGHT_DRIVING_CAN_ID    = 12;
    public static int FRONT_RIGHT_CANCODER_ID       = 32;

    public static int BACK_LEFT_TURNING_CAN_ID      = 21;
    public static int BACK_LEFT_DRIVING_CAN_ID      = 11;
    public static int BACK_LEFT_CANCODER_ID         = 31;

    public static int BACK_RIGHT_TURNING_CAN_ID     = 23;
    public static int BACK_RIGHT_DRIVING_CAN_ID     = 13;
    public static int BACK_RIGHT_CANCODER_ID        = 33;

    public static final double kWheelRadius         = 0.0508;
    public static final int kDriveEncoderResolution = 4096;

    public final static int driveEncoderTimeSamplemS = 100;
    public final static double encoderDistancePerPulse = (2 * Math.PI * kWheelRadius / kDriveEncoderResolution);
    public final static double encoderMetersPerSecondScale = encoderDistancePerPulse * (1000 / driveEncoderTimeSamplemS);
}    