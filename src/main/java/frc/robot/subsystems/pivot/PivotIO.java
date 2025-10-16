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
    double voltageSupplied = 0.0;
    double statorCurrentAmps = 0.0;
    double supplyCurrentAmps = 0.0;
    double angleMotorClosedLoopErrorAngleDegrees = 0.0;
    double closedLoopReferenceAngleDegrees = 0.0;   
    double angleMotorTemperatureCelsius = 0.0;
    double positionAngleDegrees = 0.0;  
    
    boolean connected = false;
  }

  public default void setVoltage(double voltage) {}
  public default void setAngle(Angle angle) {}
}
