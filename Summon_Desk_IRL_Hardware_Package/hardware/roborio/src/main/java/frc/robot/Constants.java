package frc.robot;

public final class Constants {
    private Constants() {}
    // CAN IDs match REV's MAXSwerve template and firmware/two-module-test.
    public static final int FRONT_LEFT_DRIVE_CAN = 11;
    public static final int FRONT_LEFT_STEER_CAN = 10;
    public static final int FRONT_RIGHT_DRIVE_CAN = 15;
    public static final int FRONT_RIGHT_STEER_CAN = 14;
    public static final int REAR_LEFT_DRIVE_CAN = 13;
    public static final int REAR_LEFT_STEER_CAN = 12;
    public static final int REAR_RIGHT_DRIVE_CAN = 17;
    public static final int REAR_RIGHT_STEER_CAN = 16;
    public static final double FRONT_LEFT_ENCODER_OFFSET_DEG = 0.0;
    public static final double FRONT_RIGHT_ENCODER_OFFSET_DEG = 0.0;
    public static final double REAR_LEFT_ENCODER_OFFSET_DEG = 0.0;
    public static final double REAR_RIGHT_ENCODER_OFFSET_DEG = 0.0;
    public static final double NT_STALE_TIMEOUT_SEC = 0.5;
}
