package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.Field2d.AlgaePosition;
import frc.robot.operator_interface.OISelector;
import frc.robot.subsystems.elevator.ElevatorConstants.ScoringHeight;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private ScoringHeight targetPosition = ScoringHeight.HARDSTOP;
    private Transform2d distanceFromReef = new Transform2d(); //distance from the reef to elevator, FIXME: change later
    private Alert hardstopAlert = new Alert("Elevator has reached hardstop!", AlertType.kWarning);
    private Alert jammedAlert = new Alert("Elevator appears to be jammed!", AlertType.kError);

    private boolean hasBeenZeroed = false;

    private final LinearFilter jamFilter = LinearFilter.singlePoleIIR(0.0, 0.00);
 
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    public Elevator(ElevatorIO io) {
        this.io = io;
        FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());
    }
 
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);
 
        Logger.getInstance().recordOutput(SUBSYSTEM_NAME + "/targetPosition", targetPosition);
        Logger.getInstance().recordOutput(SUBSYSTEM_NAME + "/distanceFromReef", distanceFromReef);
 
        if (jamFilter.calculate(Math.abs(inputs.statorCurrentAmpsLead)) > JAMMED_CURRENT) {
            CommandScheduler.getInstance()
                .schedule(
                    Commands.sequence(
                        Commands.runOnce(() -> io.setMotorVoltage(0.0), this),
                        Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ELEVATOR_JAMMED))
                            .withTimeout(1.0))
                        .withName("stop elevator jammed"));
            jammedAlert.set(true);
        } else {
            jammedAlert.set(false);
        }
    }
 
    public Command getElevatorSystemCheckCommand() {
        return Commands.sequence(
                getTestPositionCommand(ScoringHeight.L1),
                getTestPositionCommand(ScoringHeight.ABOVE_L1),
                getTestPositionCommand(ScoringHeight.L2),
                getTestPositionCommand(ScoringHeight.L3),
                getTestPositionCommand(ScoringHeight.L4),
                getTestPositionCommand(ScoringHeight.MAX_L2),
                getTestPositionCommand(ScoringHeight.MAX_L3),
                getTestPositionCommand(ScoringHeight.BELOW_LOW_ALGAE),
                getTestPositionCommand(ScoringHeight.LOW_ALGAE),
                getTestPositionCommand(ScoringHeight.BELOW_HIGH_ALGAE),
                getTestPositionCommand(ScoringHeight.HIGH_ALGAE),
                getTestPositionCommand(ScoringHeight.BARGE),
                getTestPositionCommand(ScoringHeight.PROCESSOR))
            .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
            .andThen(getElevatorLowerAndResetCommand())
            .withName(SUBSYSTEM_NAME + "SystemCheck");
    }

    private Command getTestPositionCommand(ScoringHeight reefBranch) {
        return Commands.sequence(
            Commands.runOnce(() -> goToPosition(reefBranch)),
            Commands.waitUntil(() -> isAtPosition(reefBranch))
            );      
    }

    public Distance getHeightForScoringPosition(ScoringHeight position) {
        switch (position) {
            case L1:
                return L1_HEIGHT;
            case L2:
                return L2_HEIGHT;
            case L3:
                return L3_HEIGHT;
            case L4:
                return L4_HEIGHT;
            default:
                return MIN_HEIGHT;
        }
    }
    
    public boolean isAtPosition(ScoringHeight reefBranch) {
        return atPosition(reefBranch);
    }
    
    public boolean isAtTargetPosition() {
        Distance targetHeight = getHeightForScoringPosition(targetPosition);
        return Math.abs(inputs.positionInches - targetHeight.inches()) < POSITION_TOLERANCE.inches(); 
    }
    
    public void setTargetPosition(ScoringHeight position) {
        this.targetPosition = position;
        Distance targetHeight = getHeightForScoringPosition(position);
        io.setPosition(targetHeight);
    }
    
    public void gotoPosition(ScoringHeight position) {
        setTargetPosition(position);
    }
    
    public boolean atPosition(ScoringHeight position) {
        Distance target = getHeightForScoringPosition(position);
        return Math.abs(inputs.positionInches - target.inches()) < POSITION_TOLERANCE.inches();
    }
    
    public void zeroPosition() {
        io.zeroPosition();
        hasBeenZeroed = true;
    }
}