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
  // then there are other states that can do that.
  private State lastState = State.UNITIALIZED;

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


    // 3061 example subsystem with a lot of errors
    private enum State {
      WAITING_FOR_L1_CORAL_IN_FUNNEL
      {
        @Override
        void onEnter(Manipulator subsystem) 
        {
          subsystem.setFunnelMotorVoltage(subsystem.funnelCollectionVoltage.get());
          subsystem.setIndexerMotorVoltage(subsystem.indexerCollectionVoltage.get());
          subsystem.readyToScore = false;
        }
  
        @Override
        void execute(Manipulator subsystem) {
  
          LEDs.getInstance().requestState(States.WAITING_FOR_CORAL);
          subsystem.retractPivot();
  
          if (subsystem.disableFunnelForClimb) {
            subsystem.setFunnelMotorVoltage(0.0);
            subsystem.disableFunnelForClimb =
                false; // set to false so we don't periodically request 0 voltage
          }
  
          if (subsystem.inputs.isFunnelIRBlocked) {
            subsystem.setState(State.INDEXING_CORAL_IN_MANIPULATOR);
          } else if (DriverStation.isDisabled() && subsystem.inputs.isIndexerIRBlocked) {
            subsystem.setState(State.CORAL_IN_MANIPULATOR);
          } else if (subsystem.inputs.isIndexerIRBlocked) {
            subsystem.setState(State.INDEXING_CORAL_IN_MANIPULATOR);
          } else if (subsystem.intakeAlgaeButtonPressed) {
            subsystem.setState(State.WAITING_FOR_ALGAE);
          } else if (subsystem.inputs.isAlgaeIRBlocked) {
            subsystem.setState(State.ALGAE_IN_MANIPULATOR);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      INDEXING_CORAL_IN_MANIPULATOR 
      {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setFunnelMotorVoltage(subsystem.funnelCollectionVoltage.get());
          subsystem.setIndexerMotorVoltage(subsystem.indexerCollectionVoltage.get());
          subsystem.coralInIndexingState.restart(); // start timer
          subsystem.currentInAmps
              .reset(); // reset the linear filter thats used to detect a current spike
        }
  
        @Override
        void execute(Manipulator subsystem) {
  
          LEDs.getInstance().requestState(States.INDEXING_CORAL);
          subsystem.retractPivot();
  
          if (subsystem.inputs.isIndexerIRBlocked
              && subsystem.currentInAmps.lastValue()
                  > CORAL_CURRENT_SPIKE_THRESHOLD) // the currentInAmps filters out the current in
          // the
          // noise and getting the lastValue gets the last value
          // of the current, and if that last value is greater
          // than some constant, then current spike has been
          // detected
          {
            subsystem.setState(State.CORAL_IN_MANIPULATOR);
          } else if (subsystem.coralInIndexingState.hasElapsed(
              CORAL_COLLECTION_TIME_OUT)) // hasElapsed method check if the timer has elapsed a
          // certain number of seconds
          {
            subsystem.setState(CORAL_STUCK);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      CORAL_STUCK {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setFunnelMotorVoltage(
              subsystem.funnelEjectingVoltage.get()); // set negative velocity to funnel motor to
          // invert it
          subsystem.setIndexerMotorVoltage(
              subsystem.indexerEjectingVoltage.get()); // set negative velocity to indexer motor
          // to invert it
          subsystem.ejectingCoralTimer.restart();
        }
  
        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.EJECTING_CORAL);
          subsystem.retractPivot();
  
          if (subsystem.inputs.isIndexerIRBlocked) {
            subsystem.setState(State.CORAL_IN_MANIPULATOR);
          } else if (!subsystem.inputs.isFunnelIRBlocked
              && !subsystem.inputs.isIndexerIRBlocked
              && subsystem.ejectingCoralTimer.hasElapsed(FINAL_EJECT_CORAL_DURATION_SECONDS)) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          } else if (subsystem.ejectingCoralTimer.hasElapsed(SECOND_INTAKE_CORAL_SECONDS)) {
            subsystem.setFunnelMotorVoltage(subsystem.funnelEjectingVoltage.get());
            subsystem.setIndexerMotorVoltage(subsystem.indexerEjectingVoltage.get());
          } else if (subsystem.ejectingCoralTimer.hasElapsed(FIRST_EJECT_CORAL_SECONDS)) {
            subsystem.setFunnelMotorVoltage(subsystem.funnelCollectionVoltage.get());
            subsystem.setIndexerMotorVoltage(subsystem.indexerCollectionVoltage.get());
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      CORAL_IN_MANIPULATOR {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setFunnelMotorVoltage(0.0);
          subsystem.zeroIndexerPosition();
          subsystem.targetIndexerPosition = 0.0;
          subsystem.shootCoralButtonPressed = false;
          subsystem.coralInManipulatorFirstRun = true;
        }
  
        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.HAS_CORAL);
          subsystem.retractPivot();
  
          if (DriverStation.isDisabled()) {
            subsystem.targetIndexerPosition = subsystem.inputs.indexerPositionRotations;
          }
  
          if (subsystem.coralInManipulatorFirstRun) {
            subsystem.coralInManipulatorFirstRun = false;
          } else {
            subsystem.holdWheelPosition(subsystem.targetIndexerPosition);
          }
  
          Logger.recordOutput(
              SUBSYSTEM_NAME + "/targetWheelPosition", subsystem.targetIndexerPosition);
  
          if (subsystem.shootCoralButtonPressed) {
            subsystem.setState(State.SHOOT_CORAL);
            subsystem.shootCoralButtonPressed = false;
          } else if (!subsystem.inputs.isIndexerIRBlocked) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {
          /*NO-OP */
        }
      },
      SHOOT_CORAL {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.readyToScore = false;
  
          if (subsystem.shootingFast) {
            if (OISelector.getOperatorInterface().getLevel2Trigger().getAsBoolean()
                || OISelector.getOperatorInterface().getLevel3Trigger().getAsBoolean()) {
              subsystem.setIndexerMotorVoltage(subsystem.slowShootingVoltage.get());
            } else {
              subsystem.setIndexerMotorVoltage(subsystem.fastShootingVoltage.get());
            }
          } else {
            subsystem.setIndexerMotorVoltage(subsystem.fastShootingVoltage.get());
          }
        }
  
        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.SCORING);
          subsystem.retractPivot();
  
          if (!subsystem.inputs.isFunnelIRBlocked && !subsystem.inputs.isIndexerIRBlocked) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      WAITING_FOR_ALGAE {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setPivotMotorCurrent(PIVOT_EXTEND_CURRENT);
          subsystem.setFunnelMotorVoltage(0.0);
          subsystem.setIndexerMotorVoltage(INDEXER_COLLECT_ALGAE_VOLTAGE);
          subsystem.intakingAlgaeTimer.restart();
          subsystem.currentInAmps.reset();
        }
  
        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.COLLECTING_ALGAE);
  
          // check for current spike or if algae IR has detected algae
          if (subsystem.inputs.isAlgaeIRBlocked
              && subsystem.currentInAmps.lastValue() > ALGAE_CURRENT_SPIKE_THRESHOLD) {
            subsystem.setState(State.ALGAE_IN_MANIPULATOR);
          } else if (subsystem.intakingAlgaeTimer.hasElapsed(INTAKE_ALGAE_TIMEOUT)) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {
          // set the boolean that controls if algae intake button has been pressed to false
          subsystem.intakeAlgaeButtonPressed = false;
        }
      },
  
      ALGAE_IN_MANIPULATOR {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setPivotMotorCurrent(0.0);
          subsystem.setFunnelMotorVoltage(0.0);
          subsystem.setIndexerMotorCurrent(subsystem.indexerHoldAlgaeCurrent.get());
        }
  
        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.HAS_ALGAE);
  
          // check if the shootAlgae button has been pressed, if so then switch to the SHOOT_ALGAE
          // state
          if (subsystem.scoreAlgaeInBargeButtonPressed) {
            subsystem.setState(State.SHOOT_ALGAE_IN_BARGE);
          } else if (subsystem.scoreAlgaeInProcessorButtonPressed) {
            subsystem.setState(State.SHOOT_ALGAE_IN_PROCESSOR);
          } else if (subsystem.dropAlgaeButtonPressed) {
            subsystem.setState(State.DROP_ALGAE);
          } else if (!subsystem.inputs.isAlgaeIRBlocked) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      SHOOT_ALGAE_IN_BARGE { // state robot is in while algae is being shot out of the manipulator
        @Override
        void onEnter(Manipulator subsystem) {
          // set the indexer/roller motor to a negative voltage in order for the rollers to move the
          // opp direction and eject the algae out of the manipulator
  
          subsystem.setFunnelMotorVoltage(0.0);
          subsystem.setIndexerMotorCurrent(INDEXER_SHOOT_ALGAE_BARGE_CURRENT);
          subsystem.scoreAlgaeInBargeButtonPressed = false;
          subsystem.setPivotMotorCurrent(0.0);
  
          subsystem.scoringAlgaeTimer.restart();
        }
  
        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.SCORING);
  
          if (!subsystem.inputs.isAlgaeIRBlocked
              && subsystem.scoringAlgaeTimer.hasElapsed(BARGE_ALGAE_TIMEOUT)) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      SHOOT_ALGAE_IN_PROCESSOR {
        @Override
        void onEnter(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.SCORING);
  
          subsystem.setFunnelMotorVoltage(0.0);
          subsystem.setIndexerMotorCurrent(INDEXER_SHOOT_ALGAE_PROCESSOR_CURRENT);
          subsystem.scoreAlgaeInBargeButtonPressed = false;
          subsystem.scoreAlgaeInProcessorButtonPressed = false;
          subsystem.setPivotMotorCurrent(0.0);
  
          subsystem.scoringAlgaeTimer.restart();
        }
  
        @Override
        void execute(Manipulator subsystem) {
          if (!subsystem.inputs.isAlgaeIRBlocked
              && subsystem.scoringAlgaeTimer.hasElapsed(PROCESSOR_ALGAE_TIMEOUT)) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
      DROP_ALGAE {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setFunnelMotorVoltage(0.0);
          subsystem.setIndexerMotorCurrent(INDEXER_DROP_ALGAE_CURRENT);
          subsystem.dropAlgaeButtonPressed = false;
          subsystem.setPivotMotorCurrent(0.0);
  
          subsystem.scoringAlgaeTimer.restart();
        }
  
        @Override
        void execute(Manipulator subsystem) {
          if (!subsystem.inputs.isAlgaeIRBlocked
              && subsystem.scoringAlgaeTimer.hasElapsed(DROP_ALGAE_TIMEOUT)) {
            subsystem.setState(State.WAITING_FOR_CORAL);
          }
        }
  
        @Override
        void onExit(Manipulator subsystem) {}
      },
  
      UNINITIALIZED {
  
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setFunnelMotorVoltage(0);
          subsystem.setIndexerMotorVoltage(0);
          subsystem.setPivotMotorCurrent(0);
        }
  
        @Override
        void execute(Manipulator subsystem) {
          subsystem.setState(
              State.WAITING_FOR_CORAL); // default state to WAITING_FOR_CORAL_IN_FUNNEL state
        }
  
        @Override
        void onExit(Manipulator subsystem) {
          /*NO-OP */
        }

        // io methods
        public


      };
  
}














  
  