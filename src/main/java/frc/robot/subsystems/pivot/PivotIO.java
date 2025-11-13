package frc.robot.subsystems.pivot;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference). The first property is always `connected` and logs if
  // each device is reachable. Due to logging limitations, properties cannot be a subtype of
  // Measure. Therefore all properties are suffix with their unit to mitigate bugs due to unit
  // mismatches.
  @AutoLog
  public static class PivotIOInputs {

    // voltage
    double leadvoltageSupplied = 0.0;
    double follower1VoltageSupplied = 0.0;
    double follower2VoltageSupplied = 0.0;
    double follower3VoltageSupplied = 0.0;

    // stator
    double leadstatorCurrentAmps = 0.0;
    double follower1StatorCurrentAmps = 0.0;
    double follower2StatorCurrentAmps = 0.0;
    double follower3StatorCurrentAmps = 0.0;

    // supply
    double leadsupplyCurrentAmps = 0.0;
    double follower1SupplyCurrentAmps = 0.0;
    double follower2SupplyCurrentAmps = 0.0;
    double follower3SupplyCurrentAmps = 0.0;

    double closedLoopError = 0.0;
    double closedLoopReference = 0.0;

    // temperature
    double leadTempCelsius = 0.0;
    double follower1TempCelsius = 0.0;
    double follower2TempCelsius = 0.0;
    double follower3TempCelsius = 0.0;
    double angleDegrees = 0.0;

    // connections

    boolean leadMotorConnected = false;
    boolean follower1MotorConnected = false;
    boolean follower2MotorConnected = false;
    boolean follower3MotorConnected = false;
  }

  public default void setVoltage(double voltage) {}

  public default void setAngle(Angle angle) {}

  public default void updateInputs(PivotIOInputs inputs) {}
  // public Object setAngleMotorVoltage(double in);
}
