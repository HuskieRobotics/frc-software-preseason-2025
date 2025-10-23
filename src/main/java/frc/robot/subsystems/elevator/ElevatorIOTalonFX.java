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
    private StatusSignal<RPS> leadMotorVelocity;
    private StatusSignal<Position> leadMotorPosition;
    private StatusSignal<Position> leadMotorRPS;//no. of rotations
    private StatusSignal<LoopError> leadMotorLoopError;
    private StatusSignal<LoopReference> leadMotorLoopReference;

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

}