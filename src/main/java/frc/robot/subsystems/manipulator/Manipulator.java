package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.States;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OISelector;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase 
{
  private ManipulatorIO io;

  //private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  // Add testing mode tunables here if necesary!

  private State state = State.WAITING_FOR_L1_CORAL_IN_FUNNEL; 
  // We are setting it to WAITING_FOR_L1_CORAL_IN_FUNNEL but if the coral needs to be set on l2 through l4,
  // then there is other logic that can change the state to the correct state
  private State lastState = State.UNINITIALIZED;

  Timer inIndexingState = new Timer(); // assosiated with CORAL_IN_MANIPULATOR_L1 state

  // Instance variables for inputs
  private boolean shootCoralButtonPressed = false;
  // add one more for shooter

  public Manipulator(ManipulatorIO io) {
    this.io = io;

    // to do, finish the constructor code

    SysIdRoutineChooser.getInstance()
        .addOption( "Funnel Current", sysIDFunnel); // changed name from "Manipulator Current" and sysIDManipulator to "Funnel
    // Current" and sysIDFunnel

    SysIdRoutineChooser.getInstance().addOption("Indexer Current", sysIDIndexer);
    // SysIdRoutineChooser.getInstance().addOption("Pivot Voltage", sysIdPivot);

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
    // if needed, delete or commnent out the onExit() methods
  }

  private enum State {
    UNINITIALIZED {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setAlgaeVoltage(0);
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    WAITING_FOR_L1_CORAL_IN_FUNNEL {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    CORAL_IN_MANIPULATOR_L1 {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    CORAL_IN_MANIPULATOR_L2_4 {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    CENTERING_CORAL_RIGHT {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    CENTERING_CORAL_LEFT {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    STUCK_WHILE_INDEXING {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    L1_CORAL_COLLECTED {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    L2_4_CORAL_COLLECTED {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

     L1_PREPARE_TO_SCORE {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    L2_4_PREPARE_TO_SCORE {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    SHOOT_L1_CORAL {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    },

    SHOOT_L2_4_CORAL {
      @Override
      void onEnter(Manipulator subsystem) {
        
      }

      @Override
      void execute(Manipulator subsystem) {

      }

      @Override
      void onExit(Manipulator subsystem) {

      }
    };
          
    abstract void execute(Manipulator subsystem);

    abstract void onEnter(Manipulator subsystem);

    abstract void onExit(Manipulator subsystem);
}

@Override
public void periodic() {
  io.updateInputs(inputs); // inputs commented out right now
}













  
  