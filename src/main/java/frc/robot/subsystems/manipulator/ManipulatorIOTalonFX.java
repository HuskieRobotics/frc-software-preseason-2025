// copied 3061-lib code

package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput; // imported this class for the sensors
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;

public class ManipulatorIOTalonFX implements ManipulatorIO {
  // This mechanism has no close loop control; we just set the voltage directly.
  private VoltageOut indexerVoltageRequest;

  private TalonFX leftCoralMotor;
  private TalonFX rightCoralMotor;
  private TalonFX algaeMotor;

  private VoltageOut leftVoltageRequest;
  private VoltageOut rightVoltageRequest;
  private VoltageOut algaeVoltageRequest;

  private TorqueCurrentFOC leftCurrentRequest;
  private TorqueCurrentFOC rightCurrentRequest;
  private TorqueCurrentFOC algaeCurrentRequest;

  private Alert manipulatorConfigAlert =
      new Alert("Failed to apply configuration for manipulator.", AlertType.kError);
  
  //status signals for each motor

  //stator
  private StatusSignal<Current> leftCoralMotorStatorCurrentAmps;
  private StatusSignal<Current> rightCoralMotorStatorCusrrentAmps;
  private StatusSignal<Current> algaeMotorStatorCurrentAmp;

  //supply
  private StatusSignal<Current> leftCoralMotorSupplyCurrentAmps;
  private StatusSignal<Current> rightCoralMotorSupplyCurrentAmps;
  private StatusSignal<Current> algaeMotorSupplyCurrentAmps;
  
  //temp
  private StatusSignal<Temperature> leftCoralMotorTemperature;
  private StatusSignal<Temperature> rightCoralMotorTemperature;
  private StatusSignal<Temperature> algaeMotorTemperature;

  //voltage
  private StatusSignal<Voltage> leftCoralMotorVoltage;
  private StatusSignal<Voltage> rightCoralMotorVoltage;
  private StatusSignal<Voltage> algaeMotorVoltage;


  private final Debouncer leftCoralMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
  private final Debouncer rightCoralMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
  private final Debouncer algaeMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);

  public ManipulatorIOTalonFX() {

    leftCoralMotor = new TalonFX(MANIPULATOR_LEFT_MOTOR_ID, RobotConfig.getInstance().getCanBusName());
    rightCoralMotor = new TalonFX(MANIPULATOR_RIGHT_MOTOR_ID);
    algaeMotor = new TalonFX(MANIPULATOR_ALGAE_MOTOR_ID);


    leftCurrentRequest = new TorqueCurrentFOC(0.0);
    rightCurrentRequest = new TorqueCurrentFOC(0.0);
    algaeCurrentRequest = new TorqueCurrentFOC(0.0);

    leftVoltageRequest = new VoltageOut(0.0);
    rightVoltageRequest = new VoltageOut(0.0);
    algaeVoltageRequest = new VoltageOut(0.0);


    leftCoralMotorVoltage = leftCoralMotor.getMotorVoltage();
    rightCoralMotorVoltage = rightCoralMotor.getMotorVoltage();
    algaeMotorVoltage = algaeMotor.getMotorVoltage();


    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        false,
        leftCoralMotorStatorCurrentAmps,
        rightCoralMotorStatorCusrrentAmps,
        algaeMotorStatorCurrentAmp,
        leftCoralMotorVoltage,
        rightCoralMotorVoltage,
        algaeMotorVoltage,
        leftCoralMotorTemperature,
        rightCoralMotorTemperature,
        algaeMotorTemperature,
        leftCoralMotorSupplyCurrentAmps,
        rightCoralMotorSupplyCurrentAmps,
        algaeMotorSupplyCurrentAmps
        );

    configLeftCoralMotor(leftCoralMotor);
    configRightCoralMotor(rightCoralMotor);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    // Determine if the motor for the manipulator is still connected (i.e., reachable on the CAN
    // bus). We do this by verifying that none of the status signals for the device report an error.
    inputs.manipulatorConnected =
        manipulatorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                manipulatorMotorSupplyCurrent,
                manipulatorMotorStatorCurrent,
                manipulatorMotorTemp,
                manipulatorMotorVoltage,
                manipulatorMotorVelocity));

    inputs.manipulatorStatorCurrentAmps = manipulatorMotorStatorCurrent.getValueAsDouble();
    inputs.manipulatorSupplyCurrentAmps = manipulatorMotorSupplyCurrent.getValueAsDouble();
    inputs.manipulatorTempCelsius = manipulatorMotorTemp.getValueAsDouble();
    inputs.manipulatorVelocityRPS = manipulatorMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.manipulatorMotorVoltage = manipulatorMotorVoltage.getValueAsDouble();
    inputs.isManipulatorPrimaryIRBlocked = !manipulatorIRSensor.get();
    inputs.isManipulatorSecondaryIRBlocked = !backupManipulatorIRSensor.get();
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., Voltage) throughout the rest of the code to mitigate bugs due to unit
  // mismatches.
  @Override
  public void setManipulatorVoltage(Voltage volts) {
    this.manipulatorMotor.setControl(indexerVoltageRequest.withOutput(volts));
  }

  private void configLeftCoralMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = MANIPULATOR_GEAR_RATIO;

    config.MotorOutput.Inverted =
        MANIPULATOR_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(motor, config, manipulatorConfigAlert);

    
    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "manipulator motor", motor);
  }

  private void configRightCoralMotor(TalonFX motor){

  }
}
