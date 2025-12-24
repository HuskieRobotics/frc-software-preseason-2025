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

  public static final int MANIPULATOR_MOTOR_ID = 56; // what is this for??
  public static final int MANIPULATOR_IR_SENSOR_ID = 0;
  public static final int MANIPULATOR_IR_BACKUP_SENSOR_ID = 1; //aren't we doing 4 ir sensors

  // just in case we're doing 4 ir sensors, im not familiar
  public static final int FRONT_LEFT_IR_DIO = 0;
  public static final int FRONT_CENTER_IR_DIO = 1;
  public static final int FRONT_RIGHT_IR_DIO = 2;
  public static final int BACK_CENTER_IR_DIO = 3;
  

  // the following are determined based on the mechanical design of the arm
  public static final boolean LEFT_CORAL_MOTOR_INVERTED = false;
  public static final boolean RIGHT_CORAL_MOTOR_INVERTED = true;
  public static final boolean ALGAE_MOTOR_INVERTED = false;
  public static final double MANIPULATOR_GEAR_RATIO = 1.0;

  // voltages are determined empirically through tuning
  public static final double MANIPULATOR_COLLECTION_VOLTAGE = 4.0;
  public static final double MANIPULATOR_RELEASE_VOLTAGE = 4.0;
  public static final double MANIPULATOR_EJECT_VOLTAGE = -12.0;

  // used to trigger state transitions
  public static final double COLLECTION_TIME_OUT = 2.0;
  public static final double EJECT_DURATION_SECONDS = 0.5;

  // current limits are determined based on current budget for the robot
  public static final double MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT = 40;
  public static final double COLLECTION_CURRENT_SPIKE_THRESHOLD = 35.0;
}
