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
    double leadVoltageSupplied = 0.0;
    double followerSameSideVoltageSupplied = 0.0;
    double followerOppositeSideVoltageSupplied = 0.0;
    double followerOppositeSide2VoltageSupplied = 0.0;

    // stator
    double leadStatorCurrentAmps = 0.0;
    double followerSameSideStatorCurrentAmps = 0.0;
    double followerOppositeSideStatorCurrentAmps = 0.0;
    double followerOppositeSide2StatorCurrentAmps = 0.0;

    // supply
    double leadSupplyCurrentAmps = 0.0;
    double followerSameSideSupplyCurrentAmps = 0.0;
    double followerOppositeSideSupplyCurrentAmps = 0.0;
    double followerOppositeSide2SupplyCurrentAmps = 0.0;

    // degrees
    double closedLoopErrorDegrees = 0.0;
    double closedLoopReferenceDegrees = 0.0;
    double angleDegrees = 0.0;
    double angleMotorReferenceAngleDegrees = 0.0;

    // temperature
    double leadTempCelsius = 0.0;
    double followerSameSideTempCelsius = 0.0;
    double followerOppositeSideTempCelsius = 0.0;
    double followerOppositeSide2TempCelsius = 0.0;

    // connections

    boolean leadMotorConnected = false;
    boolean followerSameSideMotorConnected = false;
    boolean followerOppositeSideMotorConnected = false;
    boolean followerOppositeSide2MotorConnected = false;
  }

  public default void setVoltage(double voltage) {}

  public default void setAngle(Angle angle) {}

  public default void updateInputs(PivotIOInputs inputs) {}

  // public Object setAngleMotorVoltage(double in) {}
}
