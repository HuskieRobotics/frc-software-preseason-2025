package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;

public class PivotIOTalonFX implements PivotIO {

  private TalonFX leadPivotMotor;
  private TalonFX followerPivotMotor1;
  private TalonFX followerPivotMotor2;
  private TalonFX followerPivotMotor3;

  private CANcoder pivotEncoder;

  // We usually use MotionMagic Expo voltage to control the position of a mechanism.
  private MotionMagicExpoVoltage pivotLeadMotorPositionRequest;
  private VoltageOut pivotLeadMotorVoltageRequest;

  // Creating status signals for each motor
  private StatusSignal<Voltage> voltageSuppliedLead;
  private StatusSignal<Voltage> voltageSuppliedFollower1;
  private StatusSignal<Voltage> voltageSuppliedFollower2;
  private StatusSignal<Voltage> voltageSuppliedFollower3;

  private StatusSignal<Current> leadStatorCurrent;
  private StatusSignal<Current> follower1StatorCurrent;
  private StatusSignal<Current> follower2StatorCurrent;
  private StatusSignal<Current> follower3StatorCurrent;

  private StatusSignal<Current> leadSupplyCurrent;
  private StatusSignal<Current> follower1SupplyCurrent;
  private StatusSignal<Current> follower2SupplyCurrent;
  private StatusSignal<Current> follower3SupplyCurrent;

  private StatusSignal<Temperature> leadTemperature;
  private StatusSignal<Temperature> follower1Temperature;
  private StatusSignal<Temperature> follower2Temperature;
  private StatusSignal<Temperature> follower3Temperature;

  private StatusSignal<Angle> pivotAngleDegrees;

  private double angleMotorReferenceAngleDegrees = 0.0;

  private final Debouncer connectedLeadDebouncer = new Debouncer(0.5);
  private final Debouncer connectedFollower1Debouncer = new Debouncer(0.5);
  private final Debouncer connectedFollower2Debouncer = new Debouncer(0.5);
  private final Debouncer connectedFollower3Debouncer = new Debouncer(0.5);

  private ArmSystemSim angleMotorSim;

  private Alert configAlert =
      new Alert("Failed to apply configuration for pivot.", AlertType.kError);

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


  public PivotIOTalonFX() {
    leadPivotMotor = new TalonFX(PIVOT_LEAD_MOTOR_ID);
    followerPivotMotor1 = new TalonFX(PIVOT_FOLLOWER_MOTOR_ID_1);
    followerPivotMotor2 = new TalonFX(PIVOT_FOLLOWER_MOTOR_ID_2);
    followerPivotMotor3 = new TalonFX(PIVOT_FOLLOWER_MOTOR_ID_3);

    pivotLeadMotorVoltageRequest = new VoltageOut(0.0);

    leadStatorCurrent = leadPivotMotor.getStatorCurrent();
    follower1StatorCurrent = followerPivotMotor1.getStatorCurrent();
    follower2StatorCurrent = followerPivotMotor2.getStatorCurrent();
    follower3StatorCurrent = followerPivotMotor3.getStatorCurrent();

    leadSupplyCurrent = leadPivotMotor.getSupplyCurrent();
    follower1SupplyCurrent = followerPivotMotor1.getSupplyCurrent();
    follower2SupplyCurrent = followerPivotMotor2.getSupplyCurrent();
    follower3SupplyCurrent = followerPivotMotor3.getSupplyCurrent();

    leadTemperature = leadPivotMotor.getDeviceTemp();
    follower1Temperature = followerPivotMotor1.getDeviceTemp();
    follower2Temperature = followerPivotMotor2.getDeviceTemp();
    follower3Temperature = followerPivotMotor3.getDeviceTemp();

    voltageSuppliedLead = leadPivotMotor.getMotorVoltage();
    voltageSuppliedFollower1 = followerPivotMotor1.getMotorVoltage();
    voltageSuppliedFollower2 = followerPivotMotor2.getMotorVoltage();
    voltageSuppliedFollower3 = followerPivotMotor3.getMotorVoltage();

    pivotAngleDegrees = leadPivotMotor.getPosition();

    Phoenix6Utils.registerSignals(
        false,
        leadStatorCurrent,
        follower1StatorCurrent,
        follower2StatorCurrent,
        follower3StatorCurrent,
        leadSupplyCurrent,
        follower1SupplyCurrent,
        follower2SupplyCurrent,
        follower3SupplyCurrent,
        leadTemperature,
        follower1Temperature,
        follower2Temperature,
        follower3Temperature,
        voltageSuppliedLead,
        voltageSuppliedFollower1,
        voltageSuppliedFollower2,
        voltageSuppliedFollower3);

        pivotLeadMotorPositionRequest = new MotionMagicExpoVoltage(0);
        pivotLeadMotorVoltageRequest = new VoltageOut(0);

        configPivotMotorLead(pivotMotorLead);
        configPivotMotorFollower1(pivotMotorFollower1);
        configPivotMotorFollower2(pivotMotorFollower2);
        configPivotMotorFollower3(pivotMotorFollower3);

        // Followers 2 and 3 are inverted from the rotation of Lead and Follower 1
        pivotMotorFollower1.setControl(new Follower(pivotMotorLead.getDeviceID(), true));
        pivotMotorFollower2.setControl(new Follower(pivotMotorLead.getDeviceID(), true));
        pivotMotorFollower3.setControl(new Follower(pivotMotorLead.getDeviceID(), true));

        pivotSystemSim =
            new PivotSystemSim( // FIXME: May need more params?
                pivotMotorLead,
                PivotConstants.PIVOT_MOTOR_INVERTED,
                PivotConstants.ANGLE_MOTOR_GEAR_RATIO, // FIXME: This may not be the right one
                PivotConstants.PIVOT_MASS_KG, // FIXME: Set to 0 for now
                PivotConstants.LOWER_ANGLE_LIMIT,
                PivotConstants.UPPER_ANGLE_LIMIT,
                PivotConstants.SUBSYSTEM_NAME)
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    // Determine if motors are still connected (reachable on CAN bus). If they are not they return an error.
        inputs.connectedLead =
            connectedLeadDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    voltageSuppliedLead,
                    leadStatorCurrent,
                    leadSupplyCurrent,
                    leadTemperature,
                    pivotAngleDegrees));
        inputs.connectedFollower1.calculate(
            connectedFollower1Debouncer.isAllGood(
                voltageSuppliedFollower1,
                follower1StatorCurrent,
                follower1SupplyCurrent,
                follower1Temperature));
        inputs.connectedFollower2.calculate(
            connectedFollower2Debouncer.isAllGood(
                voltageSuppliedFollower2,
                follower2StatorCurrent,
                follower2SupplyCurrent,
                follower2Temperature));
        inputs.connectedFollower3.calculate(
            connectedFollower3Debouncer.isAllGood(
                voltageSuppliedFollower3,
                follower3StatorCurrent,
                follower3SupplyCurrent,
                follower3Temperature));

    inputs.voltageSuppliedLead = voltageSuppliedLead.getValueAsDouble();
    inputs.voltageSuppliedFollower1 = voltageSuppliedFollower1.getValueAsDouble();
    inputs.voltageSuppliedFollower2 = voltageSuppliedFollower2.getValueAsDouble();
    inputs.voltageSuppliedFollower3 = voltageSuppliedFollower3.getValueAsDouble();

    inputs.statorCurrentAmpsLead = leadStatorCurrent.getValueAsDouble();
    inputs.statorCurrentAmpsFollower1 = follower1StatorCurrent.getValueAsDouble();
    inputs.statorCurrentAmpsFollower2 = follower2StatorCurrent.getValueAsDouble();
    inputs.statorCurrentAmpsFollower3 = follower3StatorCurrent.getValueAsDouble();

    inputs.supplyCurrentAmpsLead = leadSupplyCurrent.getValueAsDouble();
    inputs.supplyCurrentAmpsFollower1 = follower1SupplyCurrent.getValueAsDouble();
    inputs.supplyCurrentAmpsFollower2 = follower2SupplyCurrent.getValueAsDouble();
    inputs.supplyCurrentAmpsFollower3 = follower3SupplyCurrent.getValueAsDouble();

    inputs.leadTempCelsius = leadTemperature.getValueAsDouble();
    inputs.follower1TempCelsius = follower1Temperature.getValueAsDouble();
    inputs.follower2TempCelsius = follower2Temperature.getValueAsDouble();
    inputs.follower3TempCelsius = follower3Temperature.getValueAsDouble();

    inputs.pivotAngleDegrees = pivotAngleDegrees.getValueAsDouble();
    
    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode. To eliminate the performance hit, only retrieve the closed loop reference
    // signals if the tuning mode is enabled. It is critical that these input values are only used
    // for tuning and not used elsewhere in the subsystem.
    if (Constants.TUNING_MODE) {
      inputs.closedLoopError = pivotMotorLead.getClosedLoopError().getValueAsDouble();
      inputs.closedLoopReference = pivotMotorLead.getClosedLoopReference().getValueAsDouble();
    }

    // In order for a tunable to be useful, there must be code that checks if its value has changed.
    // When a subsystem has multiple tunables that are related, the ifChanged method is a convenient
    // to check and apply changes from multiple tunables at once.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.pivotMotorLead.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kG = motionMagic[6];

          config.MotionMagic.MotionMagicExpo_kV = motionMagic[7];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[8];

          //config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[9]; 
          // FIXME: Unsure if this is needed, probably not

          this.pivotMotorLead.getConfigurator().apply(config);
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG,
        kVExpo,
        kAExpo/*,
        cruiseVelocity*/);

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
        pivotLeadMotorPositionRequest.withPosition(); // FIXME: Unsure of how angle is handled
    )
  }

  private void configPivotMotorLead(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = PivotConstants.ANGLE_MOTOR_PEAK_CURRENT_DURATION;
    config.CurrentLimits.SupplyCurrentLowerLimit = PivotConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT; // FIXME: ? It could be OK that these are the same, unsure
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = PivotConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
  }

}
