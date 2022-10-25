package frc.robot;

public class Constants {
    public int FRONT_LEFT_DRIVER_CAN_ID = 0;
    public int FRONT_LEFT_DRIVEN_CAN_ID = 1;

    public int FRONT_RIGHT_DRIVER_CAN_ID = 2;
    public int FRONT_RIGHT_DRIVEN_CAN_ID = 3;

    public int BACK_LEFT_DRIVER_CAN_ID = 4;
    public int BACK_LEFT_DRIVEN_CAN_ID = 5;

    public int BACK_RIGHT_DRIVER_CAN_ID = 6;
    public int BACK_RIGHT_DRIVEN_CAN_ID = 7;

    public static final double kWheelRadius = 0.0508;
    public static final int kDriveEncoderResolution = 4096;

    public final static int driveEncoderTimeSamplemS = 100;
    public final static double encoderDistancePerPulse = (2 * Math.PI * kWheelRadius / kDriveEncoderResolution);
    public final static double encoderMetersPerSecondScale = encoderDistancePerPulse * (1000 / driveEncoderTimeSamplemS);
}    