package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;


public class ManipulatorConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ManipulatorConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Manipulator";

  //ids
  public static final int MANIPULATOR_LEFT_MOTOR_ID = 55;
  public static final int MANIPULATOR_RIGHT_MOTOR_ID = 54;
  public static final int MANIPULATOR_ALGAE_MOTOR_ID = 57;

  //CAN ID's for the canrange sensors
  public static final int FRONT_LEFT_SENSOR_ID = 14;
  public static final int FRONT_CENTER_SENSOR_ID = 0; //FIXME: unsure of ID
  public static final int FRONT_RIGHT_SENSOR_ID = 17;
  public static final int BACK_CENTER_SENSOR_ID = 0; //FIXME: unsure of ID

  // the following are determined based on the mechanical design of the arm
  public static final boolean LEFT_CORAL_MOTOR_INVERTED = false;
  public static final boolean RIGHT_CORAL_MOTOR_INVERTED = true;
  public static final boolean ALGAE_MOTOR_INVERTED = false;

  // gear ratios for the motors
  public static final double CORAL_MOTORS_GEAR_RATIO = 1.0;
  public static final double ALGAE_MOTOR_GEAR_RATIO = 1.0;


  /*
  * voltages are determined empirically through tuning
  */

  // voltage constants for the left coral motor
  public static final double LEFT_CORAL_MOTOR_COLLECTION_VOLTAGE = 4.0;
  public static final double LEFT_CORAL_MOTOR_RELEASE_VOLTAGE = 4.0;
  public static final double LEFT_CORAL_MOTOR_EJECT_VOLTAGE = -12.0;

  // voltage constants for the right coral motor
  public static final double RIGHT_CORAL_MOTOR_COLLECTION_VOLTAGE = 4.0;
  public static final double RIGHT_CORAL_MOTOR_RELEASE_VOLTAGE = 4.0;
  public static final double RIGHT_CORAL_MOTOR_EJECT_VOLTAGE = -12.0;

  // voltage constants for the algae motor
  public static final double ALGAE_MOTOR_COLLECTION_VOLTAGE = 6.0;
  public static final double ALGAE_MOTOR_RELEASE_VOLTAGE = -6.0;
  public static final double ALGAE_MOTOR_EJECT_VOLTAGE = -12.0;

  // used to trigger state transitions
  public static final double COLLECTION_TIME_OUT = 2.0;
  public static final double EJECT_DURATION_SECONDS = 0.5;

  /*
  * current limits are determined based on current budget for the robot
  */ 
  
  // current limits for the left coral motor
  public static final double LEFT_CORAL_MOTOR_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double LEFT_CORAL_MOTOR_PEAK_CURRENT_LIMIT = 40;

  // current limits for the right coral motor
  public static final double RIGHT_CORAL_MOTOR_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double RIGHT_CORAL_MOTOR_PEAK_CURRENT_LIMIT = 40;

  // current limits for the algae motor
  public static final double ALGAE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double ALGAE_MOTOR_PEAK_CURRENT_LIMIT = 40;

  // general manipulator current limits -- was previously here
  public static final double LEFT_CORAL_MOTOR_LOWER_VOLTAGE_LIMIT = 10.0; //unknown, needs to be determined through testing
  public static final double RIGHT_CORAL_MOTOR_LOWER_VOLTAGE_LIMIT = 10.0; //unknown, needs to be determined through testing

}
