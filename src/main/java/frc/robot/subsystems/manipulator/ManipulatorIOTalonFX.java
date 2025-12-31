// copied 3061-lib code

package frc.robot.subsystems.manipulator;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;

public class ManipulatorIOTalonFX implements ManipulatorIO {
  // This mechanism has no close loop control; we just set the voltage directly.
  private VoltageOut indexerVoltageRequest;

  private TalonFX leftCoralMotor;
  private TalonFX rightCoralMotor;
  private TalonFX algaeMotor;

  // This mechanism uses voltage control only, no current or velocity control.
  private VoltageOut leftVoltageRequest;
  private VoltageOut rightVoltageRequest;
  private VoltageOut algaeVoltageRequest;

  //import for the canrange sensors
  private final CANBus kCANBus = new CANBus("rio");

  //declarations for the canrange sensors below 
  private final CANrange frontLeft;
  private final CANrange frontCenter;
  private final CANrange frontRight;
  private final CANrange backCenter;

  /*
  * A config alert is needed for each device that may fail configuration. 
  * In this subsystem, that's three motors and four CANranges; so, seven alert objects in total. 
  * Otherwise, if the object is reused, a later successful configuration will overwrite an earlier failed configuration and we will never know.
  */

  private Alert leftCoralMotorAlert =
      new Alert("Failed to apply configuration for left coral motor.", AlertType.kError);
  private Alert rightCoralMotorAlert =
      new Alert("Failed to apply configuration for right coral motor.", AlertType.kError);
  private Alert algaeMotorAlert =
      new Alert("Failed to apply configuration for algae motor.", AlertType.kError);

  private Alert frontLeftAlert =
      new Alert("Failed to apply configuration for front left sensor.", AlertType.kError);
  private Alert frontCenterAlert =
      new Alert("Failed to apply configuration for front center sensor.", AlertType.kError);
  private Alert frontRightAlert =
      new Alert("Failed to apply configuration for front right sensor.", AlertType.kError);
  private Alert backCenterAlert =
      new Alert("Failed to apply configuration for back center sensor.", AlertType.kError);

 /*
  * Status signals for each motor
  */
 
  //stator
  private StatusSignal<Current> leftCoralMotorStatorCurrentAmps;
  private StatusSignal<Current> rightCoralMotorStatorCurrentAmps;
  private StatusSignal<Current> algaeMotorStatorCurrentAmps;

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

  // velocity (only for algae motor)
  private StatusSignal<AngularVelocity> algaeVelocityRPS;

  private final Debouncer leftCoralMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
  private final Debouncer rightCoralMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
  private final Debouncer algaeMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);

  public ManipulatorIOTalonFX() {

    leftCoralMotor = new TalonFX(MANIPULATOR_LEFT_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    rightCoralMotor = new TalonFX(MANIPULATOR_RIGHT_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    algaeMotor = new TalonFX(MANIPULATOR_ALGAE_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    frontLeft = new CANrange(FRONT_LEFT_SENSOR_ID, kCANBus);
    frontCenter = new CANrange(FRONT_CENTER_SENSOR_ID, kCANBus);
    frontRight = new CANrange(FRONT_RIGHT_SENSOR_ID, kCANBus);
    backCenter = new CANrange(BACK_CENTER_SENSOR_ID, kCANBus);

    leftVoltageRequest = new VoltageOut(0.0);
    rightVoltageRequest = new VoltageOut(0.0);
    algaeVoltageRequest = new VoltageOut(0.0);

    // All status signal objects initialized below for simulation
    leftCoralMotorVoltage = leftCoralMotor.getMotorVoltage();
    rightCoralMotorVoltage = rightCoralMotor.getMotorVoltage();
    algaeMotorVoltage = algaeMotor.getMotorVoltage();
    leftCoralMotorStatorCurrentAmps = leftCoralMotor.getStatorCurrent();
    rightCoralMotorStatorCurrentAmps = rightCoralMotor.getStatorCurrent();
    algaeMotorStatorCurrentAmps = algaeMotor.getStatorCurrent();

    leftCoralMotorTemperature = leftCoralMotor.getTemperature();
    rightCoralMotorTemperature = rightCoralMotor.getTemperature();
    algaeMotorTemperature = algaeMotor.getTemperature();

    leftCoralMotorSupplyCurrentAmps = leftCoralMotor.getSupplyCurrent();
    rightCoralMotorSupplyCurrentAmps = rightCoralMotor.getSupplyCurrent();
    algaeMotorSupplyCurrentAmps = algaeMotor.getSupplyCurrent();

    algaeVelocityRPS = algaeMotor.getVelocityRPS();

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        false,
        leftCoralMotorStatorCurrentAmps,
        rightCoralMotorStatorCurrentAmps,
        algaeMotorStatorCurrentAmps,
        leftCoralMotorVoltage,
        rightCoralMotorVoltage,
        algaeMotorVoltage,
        leftCoralMotorTemperature,
        rightCoralMotorTemperature,
        algaeMotorTemperature,
        leftCoralMotorSupplyCurrentAmps,
        rightCoralMotorSupplyCurrentAmps,
        algaeMotorSupplyCurrentAmps,
        algaeVelocityRPS
        );

    configLeftCoralMotor(leftCoralMotor);
    configRightCoralMotor(rightCoralMotor);
    configAlgaeMotor(algaeMotor);
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
    inputs.leftCoralMotorConnected = leftCoralMotorDebouncer.calculate(
      BaseStatusSignal.isAllGood(
          leftCoralMotorStatorCurrentAmps,
          leftCoralMotorSupplyCurrentAmps,
          leftCoralMotorTemperature,
          leftCoralMotorVoltage));

    inputs.rightCoralMotorConnected = rightCoralMotorDebouncer.calculate(
      BaseStatusSignal.isAllGood(
          rightCoralMotorStatorCurrentAmps,
          rightCoralMotorSupplyCurrentAmps,
          rightCoralMotorTemperature,
          rightCoralMotorVoltage));

    inputs.algaeMotorConnected = algaeMotorDebouncer.calculate(
      BaseStatusSignal.isAllGood(
          algaeMotorStatorCurrentAmp,
          algaeMotorSupplyCurrentAmps,
          algaeMotorTemperature,
          algaeMotorVoltage));

    inputs.leftCoralMotorStatorCurrentAmps = leftCoralMotorStatorCurrentAmps.getValueAsDouble();
    inputs.rightCoralMotorStatorCurrentAmps = rightCoralMotorStatorCurrentAmps.getValueAsDouble();
    inputs.algaeMotorStatorCurrentAmps = algaeMotorStatorCurrentAmp.getValueAsDouble();
  
    inputs.leftCoralMotorSupplyCurrentAmps = leftCoralMotorSupplyCurrentAmps.getValueAsDouble();
    inputs.rightCoralMotorSupplyCurrentAmps = rightCoralMotorSupplyCurrentAmps.getValueAsDouble();
    inputs.algaeMotorSupplyCurrentAmps = algaeMotorSupplyCurrentAmps.getValueAsDouble();
  
    inputs.leftCoralMotorTemperatureCelsius = leftCoralMotorTemperature.getValueAsDouble();
    inputs.rightCoralMotorTemperatureCelsius = rightCoralMotorTemperature.getValueAsDouble();
    inputs.algaeMotorTemperatureCelsius = algaeMotorTemperature.getValueAsDouble();
  
    inputs.leftCoralMotorVoltage = leftCoralMotorVoltage.getValueAsDouble();
    inputs.rightCoralMotorVoltage = rightCoralMotorVoltage.getValueAsDouble();
    inputs.algaeMotorVoltage = algaeMotorVoltage.getValueAsDouble();
        
    inputs.algaeVelocityRPS = algaeVelocityRPS.getValue().in(RotationsPerSecond); // I believe the motor we are calculating velocity for is algae

    //FIXME: make sure that we are updating the sensor values correctly below
    inputs.frontLeftSensorBlocked  = frontLeft.getIsDetected(false);
    inputs.frontCenterSensorBlocked = frontCenter.getIsDetected(false);
    inputs.frontRightSensorBlocked = frontRight.getIsDetected(false);
    inputs.backCenterSensorBlocked = backCenter.getIsDetected(false);

  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., Voltage) throughout the rest of the code to mitigate bugs due to unit
  // mismatches.
  @Override
  public void setLeftCoralVoltage(Voltage volts) {
    leftCoralMotor.setControl(leftVoltageRequest.withOutput(volts));
  }
  
  @Override
  public void setRightCoralVoltage(Voltage volts) {
    rightCoralMotor.setControl(rightVoltageRequest.withOutput(volts));
  }
  
  @Override
  public void setAlgaeVoltage(Voltage volts) {
    algaeMotor.setControl(algaeVoltageRequest.withOutput(volts));
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
        LEFT_CORAL_MOTOR_INVERTED
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
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Left coral motor", motor);
  }

  private void configRightCoralMotor(TalonFX motor){
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = MANIPULATOR_GEAR_RATIO;

    config.MotorOutput.Inverted =
        RIGHT_CORAL_MOTOR_INVERTED
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
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Right coral motor", motor);
  }

  private void configAlgaeMotor(TalonFX motor){
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = MANIPULATOR_GEAR_RATIO;

    config.MotorOutput.Inverted =
        ALGAE_MOTOR_INVERTED
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
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Algae motor", motor);
  }
}