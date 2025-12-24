// copied 3061-lib code

package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.ALGAE_MOTOR_INVERTED;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.BACK_CENTER_IR_DIO;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.FRONT_CENTER_IR_DIO;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.FRONT_LEFT_IR_DIO;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.FRONT_RIGHT_IR_DIO;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.LEFT_CORAL_MOTOR_INVERTED;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.MANIPULATOR_ALGAE_MOTOR_ID;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.MANIPULATOR_GEAR_RATIO;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.MANIPULATOR_LEFT_MOTOR_ID;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.MANIPULATOR_RIGHT_MOTOR_ID;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.RIGHT_CORAL_MOTOR_INVERTED;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.SUBSYSTEM_NAME;

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

  private VoltageOut leftVoltageRequest;
  private VoltageOut rightVoltageRequest;
  private VoltageOut algaeVoltageRequest;

  //current (not sure if needed, isn't this voltage based control?)
  private TorqueCurrentFOC leftCurrentRequest;
  private TorqueCurrentFOC rightCurrentRequest;
  private TorqueCurrentFOC algaeCurrentRequest;

  //import for the canrange sensors
  private final CANBus kCANBus = new CANBus("rio");

  //declarations for the canrange sensors below 
  private final CANrange frontLeft;
  private final CANrange frontCenter;
  private final CANrange frontRight;
  private final CANrange backCenter;

  private Alert manipulatorConfigAlert =
      new Alert("Failed to apply configuration for manipulator.", AlertType.kError);
  
 /*
  * Status signals for each motor
  */
 
  //stator
  private StatusSignal<Current> leftCoralMotorStatorCurrentAmps;
  private StatusSignal<Current> rightCoralMotorStatorCurrentAmps;
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

  // velocity (only for algae motor)
  private StatusSignal<AngularVelocity> algaeVelocityRPS;

  private final Debouncer leftCoralMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
  private final Debouncer rightCoralMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
  private final Debouncer algaeMotorDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);

  public ManipulatorIOTalonFX() {

    leftCoralMotor = new TalonFX(MANIPULATOR_LEFT_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    rightCoralMotor = new TalonFX(MANIPULATOR_RIGHT_MOTOR_ID);
    algaeMotor = new TalonFX(MANIPULATOR_ALGAE_MOTOR_ID);

    //FIXME: make sure that the intiializations for the sensors are done correctly
    frontLeft = new CANrange(1, kCANBus);
    frontCenter = new CANrange(2, kCANBus);
    frontRight = new CANrange(3, kCANBus);
    backCenter = new CANrange(4, kCANBus);
  
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
        rightCoralMotorStatorCurrentAmps,
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