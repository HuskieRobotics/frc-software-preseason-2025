package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SoftLimitConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class PivotIOTalonFX implements PivotIO {

  private TalonFX leadPivotMotor;
  private TalonFX followerPivotMotorSameSide;
  private TalonFX followerPivotMotorOppositeSide;
  private TalonFX followerPivotMotorOppositeSide2;
  private CANcoder pivotEncoder;


  private MotionMagicExpoVoltage pivotLeadMotorPositionRequest;
  private VoltageOut pivotLeadMotorVoltageRequest;

  private StatusSignal<Voltage> voltageSuppliedLead;
  private StatusSignal<Voltage> voltageSuppliedFollowerSameSide;
  private StatusSignal<Voltage> voltageSuppliedFollowerOppositeSide;
  private StatusSignal<Voltage> voltageSuppliedFollowerOppositeSide2;

  private StatusSignal<Current> leadStatorCurrent;
  private StatusSignal<Current> followerSameSideStatorCurrent;
  private StatusSignal<Current> followerOppositeSideStatorCurrent;
  private StatusSignal<Current> followerOppositeSide2StatorCurrent;

  private StatusSignal<Current> leadSupplyCurrent;
  private StatusSignal<Current> followerSameSideSupplyCurrent;
  private StatusSignal<Current> followerOppositeSideSupplyCurrent;
  private StatusSignal<Current> followerOppositeSide2SupplyCurrent;

  private StatusSignal<Temperature> leadTemperature;
  private StatusSignal<Temperature> followerSameSideTemperature;
  private StatusSignal<Temperature> followerOppositeSideTemperature;
  private StatusSignal<Temperature> followerOppositeSide2Temperature;

  private StatusSignal<Angle> pivotAngleDegrees;

  // private double angleMotorReferenceAngleDegrees = 0.0;, unsure what this is for?

  private final Debouncer connectedLeadDebouncer = new Debouncer(0.5);
  private final Debouncer connectedFollowerSameSideDebouncer = new Debouncer(0.5);
  private final Debouncer connectedFollowerOppositeSideDebouncer = new Debouncer(0.5);
  private final Debouncer connectedFollowerOppositeSide2Debouncer = new Debouncer(0.5);

  private ArmSystemSim pivotSystemSim;

  private Alert configAlertLead =
      new Alert("Failed to apply configuration for pivot.", AlertType.kError);
  private Alert configAlertFollowerSameSide =
      new Alert("Failed to apply configuration for pivot follower 1", AlertType.kError);
  private Alert configAlertFollowerOppositeSide =
      new Alert("Failed to apply configuration for pivot follower 2", AlertType.kError);
  private Alert configAlertFollowerOppositeSide2 =
      new Alert("Failed to apply configuration for pivot follower 3", AlertType.kError);

  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Pivot/PIVOT_KG", PivotConstants.PIVOT_KG);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Pivot/PIVOT_KS", PivotConstants.PIVOT_KS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Pivot/PIVOT_KV", PivotConstants.PIVOT_KV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Pivot/PIVOT_KA", PivotConstants.PIVOT_KA);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/PIVOT_KP", PivotConstants.PIVOT_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Pivot/PIVOT_KI", PivotConstants.PIVOT_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Pivot/PIVOT_KD", PivotConstants.PIVOT_KD);
  private final LoggedTunableNumber kAExpo =
      new LoggedTunableNumber("Pivot/PIVOT_KA_EXPO", PivotConstants.PIVOT_KA_EXPO);
  private final LoggedTunableNumber kVExpo =
      new LoggedTunableNumber("Pivot/PIVOT_KV_EXPO", PivotConstants.PIVOT_KV_EXPO);
  private final LoggedTunableNumber encoderOffset =
      new LoggedTunableNumber(
          "Pivot/ROTATION_ENCODER_MAGNET_OFFSET", PivotConstants.ROTATION_ENCODER_MAGNET_OFFSET);
  public PivotIOTalonFX() {
    
    leadPivotMotor = new TalonFX(PIVOT_LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    followerPivotMotorSameSide = new TalonFX(PIVOT_FOLLOWER_SAME_SIDE_ID, RobotConfig.getInstance().getCANBusName());
    followerPivotMotorOppositeSide = new TalonFX(PIVOT_FOLLOWER_OPPOSITE_SIDE_ID, RobotConfig.getInstance().getCANBusName());
    followerPivotMotorOppositeSide2 = new TalonFX(PIVOT_FOLLOWER_OPPOSITE_SIDE_ID_2, RobotConfig.getInstance().getCANBusName());

    pivotEncoder = new CANcoder(PIVOT_ENCODER_ID, RobotConfig.getInstance().getCANBusName());

    pivotLeadMotorVoltageRequest = new VoltageOut(0.0);

    leadStatorCurrent = leadPivotMotor.getStatorCurrent();
    followerSameSideStatorCurrent = followerPivotMotorSameSide.getStatorCurrent();
    followerOppositeSideStatorCurrent = followerPivotMotorOppositeSide.getStatorCurrent();
    followerOppositeSide2StatorCurrent = followerPivotMotorOppositeSide2.getStatorCurrent();

    leadSupplyCurrent = leadPivotMotor.getSupplyCurrent();
    followerSameSideSupplyCurrent = followerPivotMotorSameSide.getSupplyCurrent();
    followerOppositeSideSupplyCurrent = followerPivotMotorOppositeSide.getSupplyCurrent();
    followerOppositeSide2SupplyCurrent = followerPivotMotorOppositeSide2.getSupplyCurrent();

    leadTemperature = leadPivotMotor.getDeviceTemp();
    followerSameSideTemperature = followerPivotMotorSameSide.getDeviceTemp();
    followerOppositeSideTemperature = followerPivotMotorOppositeSide.getDeviceTemp();
    followerOppositeSide2Temperature = followerPivotMotorOppositeSide2.getDeviceTemp();

    voltageSuppliedLead = leadPivotMotor.getMotorVoltage();
    voltageSuppliedFollowerSameSide = followerPivotMotorSameSide.getMotorVoltage();
    voltageSuppliedFollowerOppositeSide = followerPivotMotorOppositeSide.getMotorVoltage();
    voltageSuppliedFollowerOppositeSide2 = followerPivotMotorOppositeSide2.getMotorVoltage();

    pivotAngleDegrees = leadPivotMotor.getPosition();

    Phoenix6Util.registerSignals(
        false,
        leadStatorCurrent,
        followerSameSideStatorCurrent,
        followerOppositeSideStatorCurrent,
        followerOppositeSide2StatorCurrent,
        leadSupplyCurrent,
        followerSameSideSupplyCurrent,
        followerOppositeSideSupplyCurrent,
        followerOppositeSide2SupplyCurrent,
        leadTemperature,
        followerSameSideTemperature,
        followerOppositeSideTemperature,
        followerOppositeSide2Temperature,
        voltageSuppliedLead,
        voltageSuppliedFollowerSameSide,
        voltageSuppliedFollowerOppositeSide,
        voltageSuppliedFollowerOppositeSide2);

    pivotLeadMotorPositionRequest = new MotionMagicExpoVoltage(0);
    pivotLeadMotorVoltageRequest = new VoltageOut(0);

    configPivotMotorLead(leadPivotMotor, pivotEncoder);
    configPivotMotorFollowerSameSide(followerPivotMotorSameSide);
    configPivotMotorFollowerOppositeSide(followerPivotMotorOppositeSide);
    configPivotMotorFollowerOppositeSide2(followerPivotMotorOppositeSide2);


    followerPivotMotorSameSide.setControl(new Follower(leadPivotMotor.getDeviceID(), false));
    followerPivotMotorOppositeSide.setControl(new Follower(leadPivotMotor.getDeviceID(), true));
    followerPivotMotorOppositeSide2.setControl(new Follower(leadPivotMotor.getDeviceID(), true));

    pivotSystemSim =
        new ArmSystemSim(
            leadPivotMotor,
            PivotConstants.ANGLE_MOTOR_INVERTED,
            PivotConstants.SENSOR_TO_MECHANISM_RATIO,
            0, // FIXME: Should be length, no value in PivotConstants
            PivotConstants.PIVOT_MASS_KG,
            PivotConstants.LOWER_ANGLE_LIMIT,
            PivotConstants.UPPER_ANGLE_LIMIT,
            0, // FIXME: Should be startingAngle, no value in PivotConstants
            PivotConstants.SUBSYSTEM_NAME);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.leadMotorConnected =
        connectedLeadDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                voltageSuppliedLead,
                leadStatorCurrent,
                leadSupplyCurrent,
                leadTemperature,
                pivotAngleDegrees));
    inputs.followerSameSideMotorConnected =
        connectedFollowerSameSideDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                voltageSuppliedFollowerSameSide,
                followerSameSideStatorCurrent,
                followerSameSideSupplyCurrent,
                followerSameSideTemperature));
    inputs.followerOppositeSideMotorConnected =
        connectedFollowerOppositeSideDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                voltageSuppliedFollowerOppositeSide,
                followerOppositeSideStatorCurrent,
                followerOppositeSideSupplyCurrent,
                followerOppositeSideTemperature));
    inputs.followerOppositeSide2MotorConnected =
        connectedFollowerOppositeSide2Debouncer.calculate(
            BaseStatusSignal.isAllGood(
                voltageSuppliedFollowerOppositeSide2,
                followerOppositeSide2StatorCurrent,
                followerOppositeSide2SupplyCurrent,
                followerOppositeSide2Temperature));

    inputs.leadVoltageSupplied = voltageSuppliedLead.getValueAsDouble();

    inputs.leadStatorCurrentAmps = leadStatorCurrent.getValueAsDouble();

    inputs.leadSupplyCurrentAmps = leadSupplyCurrent.getValueAsDouble();

    inputs.leadTempCelsius = leadTemperature.getValueAsDouble();
    inputs.followerSameSideTempCelsius = followerSameSideTemperature.getValueAsDouble();
    inputs.followerOppositeSideTempCelsius = followerOppositeSideTemperature.getValueAsDouble();
    inputs.followerOppositeSide2TempCelsius = followerOppositeSide2Temperature.getValueAsDouble();

    inputs.angleDegrees = pivotAngleDegrees.getValueAsDouble();

    if (Constants.TUNING_MODE) {
      inputs.closedLoopError = leadPivotMotor.getClosedLoopError().getValueAsDouble();
      inputs.closedLoopReference = leadPivotMotor.getClosedLoopReference().getValueAsDouble();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.leadPivotMotor.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kG = motionMagic[6];

          config.MotionMagic.MotionMagicExpo_kV = motionMagic[7];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[8];

          // config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[9];
          // FIXME: Unsure if this is needed, probably not

          this.leadPivotMotor.getConfigurator().apply(config);
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG,
        kVExpo,
        kAExpo /*,
               cruiseVelocity*/);

    if (encoderOffset.hasChanged(hashCode())) {
      CANcoderConfiguration config = new CANcoderConfiguration();
      this.pivotEncoder.getConfigurator().refresh(config);
      config.MagnetSensor.MagnetOffset = encoderOffset.get();
      this.pivotEncoder.getConfigurator().apply(config);
    }

    pivotSystemSim.updateSim();
  }

  @Override
  public void setVoltage(double voltage) {
    leadPivotMotor.setControl(
        pivotLeadMotorVoltageRequest.withLimitReverseMotion(false).withOutput(voltage));
  }

  @Override
  public void setAngle(Angle angle) {
    leadPivotMotor.setControl(
        pivotLeadMotorPositionRequest.withPosition(angle) // FIXME: Unsure of how angle is handled
        );
  }

  private void configPivotMotorLead(TalonFX motor, CANcoder encoder) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0; 
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; //FIXME: Double check-Clockwise or counterclockwise
    canCoderConfig.MagnetSensor.MagnetOffset = pivotEncoder.getPosition().getValueAsDouble(); 

    
    config.CurrentLimits.SupplyCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime =
        0; // FIXME: Not sure on value though 0 may be right
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = ANGLE_MOTOR_GEAR_RATIO;

    config.MotorOutput.Inverted =
        ANGLE_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // FIXME: Especially unsure about the above two statements

    SoftwareLimitSwitchConfigs limitSwitch = config.SoftwareLimitSwitch;
    limitSwitch.ForwardSoftLimitEnable = true;
    limitSwitch.ForwardSoftLimitThreshold = UPPER_ANGLE_LIMIT;
    limitSwitch.ReverseSoftLimitEnable = true;
    limitSwitch.ReverseSoftLimitThreshold = LOWER_ANGLE_LIMIT;

    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = PivotConstants.SENSOR_TO_MECHANISM_RATIO;
    config.Feedback.RotorToSensorRatio = PivotConstants.ANGLE_MOTOR_GEAR_RATIO;
    

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlertLead);
    Phoenix6Util.applyAndCheckConfiguration(encoder, canCoderConfig, configAlertLead);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, ("pivot " + motor), motor);
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, ("pivot" + encoder), encoder);
  }

  private void configPivotMotorFollowerSameSide(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime =
        0; // FIXME: Not sure on value though 0 may be right
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // FIXME: Especially unsure about the above two statements

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlertFollowerSameSide);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, ("pivot " + motor), motor);
  }

  private void configPivotMotorFollowerOppositeSide(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime =
        0; // FIXME: Not sure on value though 0 may be right
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // FIXME: Especially unsure about the above two statements

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlertFollowerOppositeSide);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, ("pivot " + motor), motor);
  }

  private void configPivotMotorFollowerOppositeSide2(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime =
        0; // FIXME: Not sure on value though 0 may be right
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // FIXME: Especially unsure about the above two statements

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlertFollowerOppositeSide2);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, ("pivot " + motor), motor);
  }
}
