package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class PivotIOTalonFX implements PivotIO {

  // We usually use MotionMagic Expo voltage to control the position of a mechanism.
    private MotionMagicExpoVoltage angleMotorPositionRequest;
    private VoltageOut angleMotorVoltageRequest;
  
    private StatusSignal<Current> angleMotorStatorCurrentStatusSignal;
    private StatusSignal<Current> angleMotorSupplyCurrentStatusSignal;
    private StatusSignal<Angle> angleMotorPositionStatusSignal;
    private StatusSignal<Temperature> angleMotorTemperatureStatusSignal;
    private StatusSignal<Voltage> angleMotorVoltageStatusSignal;
  
    private double angleMotorReferenceAngleDegrees = 0.0;
  
    private final Debouncer connectedDebouncer = new Debouncer(0.5);
  
    private ArmSystemSim angleMotorSim;
  
    private Alert configAlert = new Alert("Failed to apply configuration for arm.", AlertType.kError);
  
    // The following enables tuning of the PID and feedforward values for the arm by changing values
    // via AdvantageScope and not needing to change values in code, compile, and re-deploy.
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

    private final LoggedTunableNumber rotationEncoderMagnetOffset =
        new LoggedTunableNumber("Pivot/ROTATION_ENCODER_MAGNET_OFFSET", PivotConstants.ROTATION_ENCODER_MAGNET_OFFSET);
    private final LoggedTunableNumber kVExpo =
        new LoggedTunableNumber("Pivot/PIVOT_KV_EXPO", PivotConstants.PIVOT_KV_EXPO);
  
    private TalonFX angleMotor;
    private CANcoder angleEncoder;
  
    public PivotIOTalonFX() {
  
      angleMotor = new TalonFX(PIVOT_LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleEncoder = new CANcoder(PIVOT_ENCODER_ID, RobotConfig.getInstance().getCANBusName());

    angleMotorPositionRequest = new MotionMagicExpoVoltage(0);
    angleMotorVoltageRequest = new VoltageOut(0);

    angleMotorPositionStatusSignal = angleMotor.getPosition();
    angleMotorStatorCurrentStatusSignal = angleMotor.getStatorCurrent();
    angleMotorSupplyCurrentStatusSignal = angleMotor.getSupplyCurrent();
    angleMotorTemperatureStatusSignal = angleMotor.getDeviceTemp();
    angleMotorVoltageStatusSignal = angleMotor.getMotorVoltage();

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        angleMotorPositionStatusSignal,
        angleMotorStatorCurrentStatusSignal,
        angleMotorSupplyCurrentStatusSignal,
        angleMotorTemperatureStatusSignal,
        angleMotorVoltageStatusSignal);

    configAngleMotor(angleMotor, angleEncoder);

    // Create a simulation object for the arm. The specific parameters for the simulation
    // are determined based on the mechanical design of the arm. The ArmSystemSim class creates a
    // Mechanism2d that can be visualized in AdvantageScope to test code in simulation when the
    // physical mechanism is not available.
    this.angleMotorSim =
        new ArmSystemSim(
            angleMotor,
            angleEncoder,
            PivotConstants.ANGLE_MOTOR_INVERTED,
            PivotConstants.SENSOR_TO_MECHANISM_RATIO,
            PivotConstants.ANGLE_MOTOR_GEAR_RATIO,
            Units.inchesToMeters(20.0),
            Units.lbsToKilograms(20.0),
            Units.degreesToRadians(10.0),
            Units.degreesToRadians(120.0),
            Units.degreesToRadians(10.0),
            SUBSYSTEM_NAME);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    // Determine if the motor for the arm is still connected (i.e., reachable on the CAN bus). We do
    // this by verifying that none of the status signals for the device report an error.
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                angleMotorPositionStatusSignal,
                angleMotorStatorCurrentStatusSignal,
                angleMotorSupplyCurrentStatusSignal,
                angleMotorTemperatureStatusSignal,
                angleMotorVoltageStatusSignal));

    inputs.statorCurrentAmps = angleMotorStatorCurrentStatusSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = angleMotorSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.voltageSupplied = angleMotorVoltageStatusSignal.getValueAsDouble();
    inputs.angleDegrees =
        Units.rotationsToDegrees(angleMotorPositionStatusSignal.getValueAsDouble());
    inputs.angleMotorTemperatureCelsius = angleMotorTemperatureStatusSignal.getValueAsDouble();
    inputs.angleMotorReferenceAngleDegrees = this.angleMotorReferenceAngleDegrees;

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode. To eliminate the performance hit, only retrieve the closed loop reference
    // signals if the tuning mode is enabled. It is critical that these input values are only used
    // for tuning and not used elsewhere in the subsystem. For example, the
    // angleMotorReferenceAngleDegrees property should be used throughout the subsystem since it
    // will always be populated.
    if (Constants.TUNING_MODE) {
      inputs.closedLoopReferenceAngleDegrees =
          Units.rotationsToDegrees(angleMotor.getClosedLoopReference().getValueAsDouble());
      inputs.angleMotorClosedLoopErrorAngleDegrees =
          Units.rotationsToDegrees(angleMotor.getClosedLoopError().getValueAsDouble());
    }

    // In order for a tunable to be useful, there must be code that checks if its value has changed.
    // When a subsystem has multiple tunables that are related, the ifChanged method is a convenient
    // to check and apply changes from multiple tunables at once.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.angleMotor.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kG = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kV = motionMagic[6];
          config.MotionMagic.MotionMagicExpo_kV = motionMagic[7];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[8];
          this.angleMotor.getConfigurator().apply(config);
        }
       );

    
    // The last step in the updateInputs method is to update the simulation.
    this.angleMotorSim.updateSim();
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., Angle) throughout the rest of the code to mitigate bugs due to unit
  // mismatches. When calling into the Phoenix API, we may need to convert from Measure types to
  // doubles of the expected unit (e.g., rotations).
  @Override
  public void setAngle(Angle angle) {
    angleMotor.setControl(angleMotorPositionRequest.withPosition(angle.in(Rotations)));

    // To improve performance, we store the reference angle as an instance variable to avoid having
    // to retrieve the status signal object from the device in the updateInputs method.
    this.angleMotorReferenceAngleDegrees = angle.in(Degrees);
  }

  @Override
  public void setVoltage(double voltage) {
    angleMotor.setControl(angleMotorVoltageRequest.withOutput(voltage));
  }

  private void configAngleMotor(TalonFX angleMotor, CANcoder angleEncoder) {

    CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
    angleCANCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    angleCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    angleCANCoderConfig.MagnetSensor.MagnetOffset = rotationEncoderMagnetOffset.get();
    Phoenix6Util.applyAndCheckConfiguration(angleEncoder, angleCANCoderConfig, configAlert);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();

    angleMotorConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
        PivotConstants.ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
        PivotConstants.ANGLE_MOTOR_PEAK_CURRENT_DURATION;
    angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleMotorConfig.CurrentLimits.StatorCurrentLimit = PivotConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotorConfig.Slot0.kP = kP.get();
    angleMotorConfig.Slot0.kI = kI.get();
    angleMotorConfig.Slot0.kD = kD.get();
    angleMotorConfig.Slot0.kS = kS.get();
    angleMotorConfig.Slot0.kG = kG.get();
    angleMotorConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
    angleMotorConfig.Slot0.kA = kA.get();
    angleMotorConfig.Slot0.kV = kV.get();

    angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    angleMotorConfig.MotionMagic.MotionMagicExpo_kV = kAExpo.get();
    angleMotorConfig.MotionMagic.MotionMagicExpo_kA = kVExpo.get();

    angleMotorConfig.MotorOutput.Inverted =
        PivotConstants.ANGLE_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Software limit switches are used to prevent the arm from moving beyond its physical limits.
    SoftwareLimitSwitchConfigs angleMotorLimitSwitches = angleMotorConfig.SoftwareLimitSwitch;
    angleMotorLimitSwitches.ForwardSoftLimitEnable = true;
    angleMotorLimitSwitches.ForwardSoftLimitThreshold =
        Units.degreesToRotations(PivotConstants.UPPER_ANGLE_LIMIT);
    angleMotorLimitSwitches.ReverseSoftLimitEnable = true;
    angleMotorLimitSwitches.ReverseSoftLimitThreshold =
        Units.degreesToRotations(PivotConstants.LOWER_ANGLE_LIMIT);

    // For the most accurate measurement of the arm's position, fuse the CANcoder with the encoder
    // in the TalonFX.
    angleMotorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleMotorConfig.Feedback.SensorToMechanismRatio = PivotConstants.SENSOR_TO_MECHANISM_RATIO;
    angleMotorConfig.Feedback.RotorToSensorRatio = PivotConstants.ANGLE_MOTOR_GEAR_RATIO;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(angleMotor, angleMotorConfig, configAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleMotor", angleMotor);
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleCANcoder", angleEncoder);
  }
}
