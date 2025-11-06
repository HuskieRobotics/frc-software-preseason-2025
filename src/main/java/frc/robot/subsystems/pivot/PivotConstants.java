package frc.robot.subsystems.pivot;

public class PivotConstants {

  public static final String SUBSYSTEM_NAME = "Pivot";

  public static final int PIVOT_LEAD_MOTOR_ID = 80;
  public static final int PIVOT_FOLLOWER_MOTOR_ID_1 = 81;
  public static final int PIVOT_FOLLOWER_MOTOR_ID_2 = 83;
  public static final int PIVOT_FOLLOWER_MOTOR_ID_3 = 84;

  // FIXME: This could be 100 instead, but based on design document setting to this
  public static final double ANGLE_MOTOR_GEAR_RATIO = 123.45679;
  public static final int SENSOR_TO_MECHANISM_RATIO = 1;

  public static final boolean PIVOT_MOTOR_INVERTED = false;

  public static final double LOWER_ANGLE_LIMIT = -90;
  public static final double UPPER_ANGLE_LIMIT = 90;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_LIMIT = 40; // amps
  public static final double ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 30; // amps
  public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 200; // milliseconds
  public static final double ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE = 2.0; // volts
  public static final double ANGLE_TOLERANCE_DEGREES = 2.0; // degrees

  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 1000; // degrees per second

  public static final int PIVOT_ENCODER_ID = 0;

  public static final double PIVOT_KG = 0;
  public static final double PIVOT_KS = 0;
  public static final double PIVOT_KV = 0;
  public static final double PIVOT_KA = 0;
  public static final double PIVOT_KP = 0;
  public static final double PIVOT_KI = 0;
  public static final double PIVOT_KD = 0;
  public static final double PIVOT_KA_EXPO = 0;
  public static final double PIVOT_KV_EXPO = 0;

  public static final boolean ANGLE_MOTOR_INVERTED = false;

  public static final double ROTATION_ENCODER_MAGNET_OFFSET = 0;

  public static final double PIVOT_MASS_KG = 0; // FIXME: Change to pivot mass later
}
