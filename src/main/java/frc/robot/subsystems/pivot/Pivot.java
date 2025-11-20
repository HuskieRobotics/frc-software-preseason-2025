package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;

  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Pivot/TestingMode", 0);
  private final LoggedTunableNumber angleManualControlVoltage =
      new LoggedTunableNumber("Pivot/ManualControlVoltage", ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE);
  private final LoggedTunableNumber pivotAngleDegrees =
      new LoggedTunableNumber("Pivot/AngleDegrees", LOWER_ANGLE_LIMIT);

  // private final Debouncer atSetpointDebouncer = new Debouncer(0.1);

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setVoltage(output.in(Volts)), null, this));

  public Pivot(PivotIO io) {
    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Pivot Voltage", sysIdRoutine);

    // FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {
      if (pivotAngleDegrees.get() != 0) {
        io.setAngle(Degrees.of(pivotAngleDegrees.get()));
      } else if (angleManualControlVoltage.get() != 0) {
        io.setVoltage(angleManualControlVoltage.get());
      }
    }

    LoggedTracer.record("Pivot");
  }

  public void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public Angle getPositionAngle() {
    return Degrees.of(inputs.angleDegrees); // FIXME: We think we fixed this but not sure yet
  }
}
