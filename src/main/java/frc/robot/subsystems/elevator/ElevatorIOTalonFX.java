package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.MotionMagicExpo;

public class ElevatorIOTalonFX implements ElevatorIO(){
    //Lead motor
    private final TalonFX leadMotor; 
    private final TalonFX followerMotor;

    private VoltageOut elevatorVoltageRequest;
    private MotionMagicExpo elevatorPositionRequest;

    private Alert elevatorLeadConfigAlert =
        new Alert("Failed to apply configuration for lead elevator motor.", AlertType.kError);
    private Alert elevatorFollowerConfigAlert =
        new Alert("Failed to apply configuration for follower elevator motor.", AlertType.kError);

    private StatusSignal<Volts> leadMotorStatorCurrent;
    private StatusSignal<Volts> leadMotorVoltageSupplyCurrent;
    private StatusSignal<Amps> leadMotorSupplyCurrent;
    private StatusSignal<Temperature> leadMotorTemp;
    private StatusSignal<AngularVelocity> leadMotorVelocity;
    private StatusSignal<Angle> leadMotorPosition;
    private StatusSignal<Position> leadMotorRPS;//no. of rotations

    private final Debouncer topMotorConnectedDebouncer = new Debouncer(Constants.kMotorConnectionDebounceTimeSeconds);
    private final Debouncer followerMotorConnectedDebouncer = new Debouncer(Constants.kMotorConnectionDebounceTimeSeconds);

    private final LoggedTunableNumber elevatorLeadKg = new LoggedTunableNumber("Elevator/LeadKg", ElevatorConstants.KG);
    private final LoggedTunableNumber elevatorLeadKs = new LoggedTunableNumber("Elevator/LeadKs", ElevatorConstants.KS);
    private final LoggedTunableNumber elevatorLeadKv = new LoggedTunableNumber("Elevator/LeadKv", ElevatorConstants.KV);
    private final LoggedTunableNumber elevatorLeadKa = new LoggedTunableNumber("Elevator/LeadKa", ElevatorConstants.KA);
    private final LoggedTunableNumber elevatorLeadKp = new LoggedTunableNumber("Elevator/LeadKp", ElevatorConstants.KP);
    private final LoggedTunableNumber elevatorLeadKi = new LoggedTunableNumber("Elevator/LeadKi", ElevatorConstants.KI);
    private final LoggedTunableNumber elevatorLeadKd = new LoggedTunableNumber("Elevator/LeadKd", ElevatorConstants.KD);
    private final LoggedTunableNumber elevatorLeadKvexpo = new LoggedTunableNumber("Elevator/LeadKvExpo", ElevatorConstants.KVEXPO);
    private final LoggedTunableNumber elevatorLeadKaexpo = new LoggedTunableNumber("Elevator/LeadKaExpo", ElevatorConstants.KAEXPO);
    private final LoggedTunableNumber crusiveVelocity = new LoggedTunableNumber("Elevator/CrusiveVelocity", 0);

    public ElevatorIOTalonFX(){
    leadMotor = new TalonFX(RobotConfig.getInstance("Elevator/LeadMotorCANID", 0));//Chnage can ID later
    followerMotor = new TalonFX(RobotConfig.getInstance("Elevator/FollowerMotorCANID", 1));//Change can ID later

    leadMotorStatorCurrent = leadMotor.getStatorCurrent();
    followerMotorStatorCurrent = followerMotor.getStatorCurrent();

    leadMotorVoltageSupplyCurrent = leadMotor.getSupplyCurrent();
    followerMotorVoltageSupplyCurrent = followerMotor.getSupplyCurrent();

    leadMotorTemp = leadMotor.getDeviceTemp();
    followerMotorTemp = followerMotor.getDeviceTemp();

    leadMotorRotations = leadMotor.getRotations();
    followerMotorRotations = followerMotor.getRotations();

    Phoenix6Util.registerSignals(
        true,
        leadMotorStatorCurrent,
        followerMotorStatorCurrent,
        leadMotorVoltageSupplyCurrent,
        followerMotorVoltageSupplyCurrent,
        leadMotorTemp,
        followerMotorTemp,
        leadMotorRotations,
        followerMotorRotations
    );

    configureElevatorMotors(leadMotor);
    configureElevatorMotors(followerMotor);

    //need to know how to connect motors here
    //I don't know what else to add here gonna know it later
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leadMotorStatorCurrent = leadMotorStatorCurrent.getValue();
        inputs.followerMotorStatorCurrent = followerMotorStatorCurrent.getValue();

        inputs.leadMotorVoltageSupplyCurrent = leadMotorVoltageSupplyCurrent.getValue();
        inputs.followerMotorVoltageSupplyCurrent = followerMotorVoltageSupplyCurrent.getValue();

        inputs.leadMotorTemp = leadMotorTemp.getValue();
        inputs.followerMotorTemp = followerMotorTemp.getValue();

        inputs.leadMotorRotations = leadMotorRotations.getValue();
        inputs.followerMotorRotations = followerMotorRotations.getValue();

        ElevatorSystemSim.updateSim();

        if (Constants.TUNING_MODE) {
      inputs.leadMotorClosedLoopReferenceVelocityRPS =
          leadMotor.getClosedLoopReference().getValue();
      inputs.leadMotorClosedLoopErrorVelocityRPS =
          leadMotor.getClosedLoopError().getValue();
    }
    }

    private void configureElevatorMotors(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        MotionMagicConfig motionMagicConfig = new MotionMagicConfig();

        //Need to set the config values here & I don't know how to do that
        alonFXConfiguration config = new TalonFXConfiguration();

        MotionMagicConfigs leadMotorConfig = config.MotionMagic;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.CurrentLimits.SupplyCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted =
            IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = kPslot0.get();
        config.Slot0.kI = kIslot0.get();
        config.Slot0.kD = kDslot0.get();
        config.Slot0.kS = kSslot0.get();
        config.Slot0.kV = kVslot0.get();
        config.Slot0.kA = kAslot0.get();
        config.Slot0.kG = kGslot0.get();

        config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

        config.Slot1.kP = kPslot1.get();
        config.Slot1.kI = kIslot1.get();
        config.Slot1.kD = kDslot1.get();
        config.Slot1.kS = kSslot1.get();
        config.Slot1.kV = kVslot1.get();
        config.Slot1.kA = kAslot1.get();
        config.Slot1.kG = kGslot1.get();

        config.Slot1.withGravityType(GravityTypeValue.Elevator_Static);

        leadMotorConfig.MotionMagicExpo_kA = kAExpo.get();
        leadMotorConfig.MotionMagicExpo_kV = kVExpo.get();

        leadMotorConfig.MotionMagicCruiseVelocity = cruiseVelocity.get();

        // configure a hardware limit switch that zeros the elevator when lowered; there is no hardware
        // limit switch, but we will set it using a control request
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;

        Phoenix6Util.applyAndCheckConfiguration(elevatorMotorLead, config, leadConfigAlert);

        FaultReporter.getInstance().registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Lead", motor);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        elevatorMotorLead.setVoltage(leadVoltageRequest.withOutput(voltage));
        elevatorMotorFollower.setVoltage(followerVoltageRequest.withOutput(voltage));
    }
    @Override
    public void setPosition(Distance position) {
        elevatorMotorLead.setControl(elevatorPositionRequest.withPosition(position.in(inches) / CIRCUMFERENCE));
    }

    @Override
    public void zeroPosition() {
        // we set the reverse limit instead of directly setting the position to avoid the overhead of a
        // config call
        elevatorMotorLead.setControl(leadVoltageRequest.withLimitReverseMotion(true).withOutput(0.0));
    }
}