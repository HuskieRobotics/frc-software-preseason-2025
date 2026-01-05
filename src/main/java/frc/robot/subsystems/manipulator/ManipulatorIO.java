package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  //
  @AutoLog
  public static class ManipulatorIOInputs {

    //This is recording the values of the left, right, and coral motors

    //angle for "wrist position"
    double algaeMotorAngleDeg = 0;

    //stator
    double leftCoralMotorStatorCurrentAmps = 0;
    double rightCoralMotorStatorCurrentAmps = 0;
    double algaeMotorStatorCurrentAmps = 0;
    
    //supply
    double leftCoralMotorSupplyCurrentAmps = 0;
    double rightCoralMotorSupplyCurrentAmps = 0;
    double algaeMotorSupplyCurrentAmps = 0;

    //temp
    double leftCoralMotorTemperatureCelsius = 0;
    double rightCoralMotorTemperatureCelsius = 0;
    double algaeMotorTemperatureCelsius = 0;

    //voltage
    double leftCoralMotorVoltage = 0;
    double rightCoralMotorVoltage = 0;
    double algaeMotorVoltage = 0;

    // velocity (only for algae motor)
    double algaeVelocityRPS = 0;

    //connection status
    boolean leftCoralMotorConnected = false;
    boolean rightCoralMotorConnected = false;
    boolean algaeMotorConnected = false;

    //canrange sensor values
    public boolean frontLeftSensorBlocked = false;
    public boolean frontCenterSensorBlocked = false;
    public boolean frontRightSensorBlocked = false;
    public boolean backCenterSensorBlocked = false;
    
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setLeftCoralVoltage(Voltage volts) {}

  public default void setRightCoralVoltage(Voltage volts) {}

  public default void setAlgaeVoltage(Voltage volts) {}

  public default void setAlgaeAngle(Angle deg) {

}
