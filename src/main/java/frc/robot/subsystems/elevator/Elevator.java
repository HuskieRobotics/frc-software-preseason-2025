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
    private Transform2d distanceFromReef = new Transform2d(); //distance from the reef to elevator, but dont know the value to keep
    private Alert hardstopAlert = new Alert("Elevator has reached hardstop!", AlertType.kWarning);
    private Alert jammedAlert = new Alert("Elevator appears to be jammed!", AlertType.kError);

    private boolean hasBeenZeroed = false;

    private LinearFilter current = LinearFilter.singlePoleIIR(0.0, 0.0);//change later

    private LinearFilter current = LinearFilter.singlePoleIIR(0.0, 0.0);//change later

    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }
}