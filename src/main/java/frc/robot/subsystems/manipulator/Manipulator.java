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
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OISelector;
import org.littletonrobotics.junction.Logger;


public class Manipulator extends SubsystemBase {

  // all subsystems receive a reference to their IO implementation when constructed
  private ManipulatorIO io;

  // all subsystems create the AutoLogged version of their IO inputs class
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

  // Coral Motor 1 (for blue and green wheels) Tunable Numbers
  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Manipulator/TestingMode", 0);
  private final LoggedTunableNumber leftCoralMotorVoltage =
      new LoggedTunableNumber("Manipulator/MotorVoltage", 0);
  public final LoggedTunableNumber leftCoralMotorCollectionVoltage =
      new LoggedTunableNumber("Manipulator/CollectionVoltage", MANIPULATOR_COLLECTION_VOLTAGE);
  public final LoggedTunableNumber leftCoralMotorReleaseVoltage =
      new LoggedTunableNumber("Manipulator/ReleaseVoltage", MANIPULATOR_RELEASE_VOLTAGE);
  public final LoggedTunableNumber leftCoralMotorEjectVoltage =
      new LoggedTunableNumber("Manipulator/Indexer/EjectVoltage", MANIPULATOR_EJECT_VOLTAGE);

  // Coral Motor 2 (for blue and green rollers) Tunable Numbers
private final LoggedTunableNumber rightCoralMotorVoltage =
  new LoggedTunableNumber("Manipulator/MotorVoltage", 0);
public final LoggedTunableNumber rightCoralMotorCollectionVoltage =
  new LoggedTunableNumber("Manipulator/CollectionVoltage", MANIPULATOR_COLLECTION_VOLTAGE);
public final LoggedTunableNumber rightCoralMotorReleaseVoltage =
  new LoggedTunableNumber("Manipulator/ReleaseVoltage", MANIPULATOR_RELEASE_VOLTAGE);
public final LoggedTunableNumber rightCoralMotorEjectVoltage =
  new LoggedTunableNumber("Manipulator/Indexer/EjectVoltage", MANIPULATOR_EJECT_VOLTAGE);

// Algae Motor (black rollers) Tunable Numbers;
  private final LoggedTunableNumber algaeMotorVoltage =
      new LoggedTunableNumber("Manipulator/MotorVoltage", 0);
  public final LoggedTunableNumber algaeMotorCollectionVoltage =
      new LoggedTunableNumber("Manipulator/CollectionVoltage", MANIPULATOR_COLLECTION_VOLTAGE);
  public final LoggedTunableNumber algaeMotorReleaseVoltage =
      new LoggedTunableNumber("Manipulator/ReleaseVoltage", MANIPULATOR_RELEASE_VOLTAGE);
  public final LoggedTunableNumber algaeMotorEjectVoltage =
      new LoggedTunableNumber("Manipulator/Indexer/EjectVoltage", MANIPULATOR_EJECT_VOLTAGE);

  // Initialize the last state to the uninitialized state and the current state to the desired
  // initial state to ensure that the onEnter method is invoked when the subsystem in constructed.
  private State state = State.WAITING_FOR_L1_CORAL_IN_FUNNEL;
  private State lastState = State.UNINITIALIZED;

  // Some state transitions are triggered by a timeout. Use Timer objects for that purpose.
  Timer inIndexingState = new Timer();
  //Timer ejectingTimer = new Timer();
  
  // Use a linear filter to detect when the game piece has stalled against the hard stop. We want to
  // use a filter to eliminate false positives due to current spikes that may occur when the motor
  // starts or when the game piece first makes contact with the manipulator.
  private LinearFilter currentInAmps = LinearFilter.singlePoleIIR(0.1, 0.02);

  // Some state transitions are triggered by the driver or operator via a button press. We don't
  // want those commands to directly change the state as that can result in a missed state
  // transition. Instead, those commands will change a variable which is monitored within the state
  // machine.
  private boolean releaseButtonPressed = false;
  private boolean ejectButtonPressed = false;

  public Manipulator(ManipulatorIO io) {

    this.io = io;

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  /*
  * State machine states  
  */

  private enum State {
    WAITING_FOR_L1_CORAL_IN_FUNNEL {
      @Override
      void onEnter(Manipulator subsystem) {
        // Set the voltage of all motors to the collection voltage.
        subsystem.setLeftCoralMotorVoltage(
            Volts.of(subsystem.leftCoralMotorCollectionVoltage.get()));

        subsystem.setRightCoralMotorVoltage(
              Volts.of(subsystem.rightCoralMotorCollectionVoltage.get()));
      }

      @Override
      void execute(Manipulator subsystem) {

        // LEDs.getInstance().requestState(States.WAITING_FOR_GAME_PIECE); FIXME: The LED transition might not be necessary right now

        // Often preloading a game piece requires a special case state transition.
        if (DriverStation.isDisabled() && (subsystem.isManipulatorFrontLeftBlocked() && subsystem.isManipulatorFrontCenterBlocked() && 
        subsystem.isManipulatorFrontRightBlocked() && subsystem.isManipulatorBackCenterBlocked()) ) {
          subsystem.setState(State.CORAL_IN_MANIPULATOR_L2);
        } // check if the game piece is detected by the manipulator
        else if (subsystem.isManipulatorFrontLeftBlocked() || subsystem.isManipulatorFrontCenterBlocked() || 
        subsystem.isManipulatorFrontRightBlocked() || subsystem.isManipulatorBackCenterBlocked()){ 
          // FIXME: enumerate all the methods in the else if block.
          if(subsystem.isManipulatorFrontLeftBlocked() && !subsystem.isManipulatorFrontRightBlocked()){
            subsystem.setState(State.CENTERING_CORAL_RIGHT);
          } else if(subsystem.isManipulatorFrontRightBlocked() && !subsystem.isManipulatorFrontLeftBlocked()) {
            subsystem.setState(State.CENTERING_CORAL_LEFT); 
          } else {
          break; // stay in the current state
          }
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    WAITING_FOR_L2_4_CORAL_IN_FUNNEL {
      @Override
      void onEnter(Manipulator subsystem) {
        // Set the voltage of all motors to the collection voltage.
        subsystem.setLeftCoralMotorVoltage(
            Volts.of(subsystem.leftCoralMotorCollectionVoltage.get()));

        subsystem.setRightCoralMotorVoltage(
            Volts.of(subsystem.rightCoralMotorCollectionVoltage.get()));
      }
      @Override
      void execute(Manipulator subsystem) {

        // Often preloading a game piece requires a special case state transition.
        if (DriverStation.isDisabled() && (subsystem.isManipulatorFrontLeftBlocked() && subsystem.isManipulatorFrontCenterBlocked() && 
        subsystem.isManipulatorFrontRightBlocked() && subsystem.isManipulatorBackCenterBlocked()) ) {
          subsystem.setState(State.CORAL_IN_MANIPULATOR_L2_4);
        } // check if the game piece is detected by the manipulator

        else if (subsystem.isManipulatorFrontLeftBlocked() || subsystem.isManipulatorFrontCenterBlocked() || 
        subsystem.isManipulatorFrontRightBlocked() || subsystem.isManipulatorBackCenterBlocked()){ 
          // FIXME: enumerate all the methods in the else if block.
          subsystem.setState(State.INDEXING_CORAL_IN_MANIPULATOR_L2_4);
        }
      }

      void onExit(Manipulator subsystem) {}
    },

    CENTERING_CORAL_IN_MANIPULATOR_RIGHT {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setLeftCoralMotorVoltage(
            Volts.of(subsystem.leftCoralMotorCollectionVoltage.get()));
        subsystem.setRightCoralMotorVoltage(
            Volts.of(subsystem.rightCoralMotorCollectionVoltage.get()));

        // If a state has a timeout, the timer must be restarted in the onEnter method.
        subsystem.inIndexingState.restart();

        // If a state has a filter, the filter must be reset in the onEnter method.
        subsystem.currentInAmps.reset();
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.INDEXING_GAME_PIECE);

        // check if the timeout has elapsed which indicates that the game piece may be stuck
        if (subsystem.inIndexingState.hasElapsed(COLLECTION_TIME_OUT)) {
          ejectGamePiece();
          subsystem.setState(GAME_PIECE_STUCK);
        }

        // Centering logic: nudge piece left or right depending on which front sensor is triggered.
        // If only the left sensor is triggered, nudge the piece to the right and vice versa.
        if (subsystem.isManipulatorFrontLeftBlocked() && !subsystem.isManipulatorFrontRightBlocked()) {
          // move piece right: run left-side rollers forward and right-side rollers (or opposing rollers)
          // slightly in reverse to bias the piece toward the right.
          subsystem.setLeftCoralMotorVoltage(
          Volts.of(subsystem.leftCoralMotorCollectionVoltage.get()));
          subsystem.setRightCoralMotorVoltage(
          Volts.of(-subsystem.rightCoralMotorCollectionVoltage.get()));
        } //FIXME: Add logic to check if the piece has gone too far left or right and transition to the appropriate state to correct it.
        if(((subsystem.isManipulatorFrontCenterBlocked() && (subsystem.isManipulatorFrontRightBlocked() && 
           subsystem.isManipulatorFrontLeftBlocked())) || subsystem.isManipulatorBackCenterBlocked()) ) {
          subsystem.setState(State.CORAL_IN_MANIPULATOR_L1);
        }
        }
      @Override
      void onExit(Manipulator subsystem) {}
    },
      
    CENTERING_CORAL_IN_MANIPULATOR_LEFT {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setLeftCoralMotorVoltage(
            Volts.of(subsystem.leftCoralMotorCollectionVoltage.get()));
        subsystem.setRightCoralMotorVoltage(
            Volts.of(subsystem.rightCoralMotorCollectionVoltage.get()));

        // If a state has a timeout, the timer must be restarted in the onEnter method.
        subsystem.inIndexingState.restart();

        // // If a state has a filter, the filter must be reset in the onEnter method.
        // subsystem.currentInAmps.reset();
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.INDEXING_GAME_PIECE);

        // check if the timeout has elapsed which indicates that the game piece may be stuck
        if (subsystem.inIndexingState.hasElapsed(COLLECTION_TIME_OUT)) {
          ejectGamePiece();
          subsystem.setState(GAME_PIECE_STUCK);
        }

        // Centering logic: nudge piece left or right depending on which front sensor is triggered.
        // If only the left sensor is triggered, nudge the piece to the right and vice versa.
        if (subsystem.isManipulatorFrontRightBlocked() && !subsystem.isManipulatorFrontLeftBlocked()) {
          // move piece left: reverse of above
          subsystem.setLeftCoralMotorVoltage(
          Volts.of(-subsystem.leftCoralMotorCollectionVoltage.get()));
          subsystem.setRightCoralMotorVoltage(
          Volts.of(subsystem.rightCoralMotorCollectionVoltage.get()));
        } //FIXME: Add logic to check if the piece has gone too far left or right and transition to the appropriate state to correct it.
        if(((subsystem.isManipulatorFrontCenterBlocked() && (subsystem.isManipulatorFrontRightBlocked() && 
           subsystem.isManipulatorFrontLeftBlocked())) || subsystem.isManipulatorBackCenterBlocked()) ) {
          subsystem.setState(State.CORAL_IN_MANIPULATOR_L1);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    INDEXING_CORAL_IN_MANIPULATOR_L2_4 {
      @Override
      void onEnter(Manipulator subsystem) {
        // Set the voltage of all motors to the collection voltage.
        subsystem.setLeftCoralMotorVoltage(
            Volts.of(subsystem.leftCoralMotorCollectionVoltage.get()));

        subsystem.setRightCoralMotorVoltage(
              Volts.of(subsystem.rightCoralMotorCollectionVoltage.get()));

        // Start the timer to check for if coral gets stuck in this state.
        inIndexingState.reset();
        inIndexingState.start();
      }

      @Override
      void execute(Manipulator subsystem) {
      if (subsystem.isManipulatorFrontCenterBlocked() && subsystem.isManipulatorBackCenterBlocked()) {
          subsystem.setState(State.CORAL_IN_MANIPULATOR_L2_4);
      }
      }

      @Override
      void onExit(Manipulator subsystem) {
      }
    },

    GAME_PIECE_STUCK {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(Volts.of(subsystem.manipulatorEjectVoltage.get()));

        // If a state has a timeout, the timer must be restarted in the onEnter method.
        subsystem.ejectingTimer.restart();
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.EJECTING_GAME_PIECE);

        // wait for the specified duration before transitioning back to the waiting for game piece
        // state to ensure that the game piece has been ejected
        if (subsystem.ejectingTimer.hasElapsed(EJECT_DURATION_SECONDS)) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    CORAL_IN_MANIPULATOR_L1 {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setLeftCoralMotorVoltage(Volts.of(LEFT_CORAL_MOTOR_LOWER_VOLTAGE_LIMIT));
        subsystem.setRightCoralMotorVoltage(Volts.of(RIGHT_CORAL_MOTOR_LOWER_VOLTAGE_LIMIT));

        // move "wrist" to scoring position

        //subsystem.setAlgaeMotorPosition(DEGREES.of(0.0)); // FIXME: set to appropriate scoring angle with a variable
      } //Removing the wrist position for now since we don't care about algae.

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.HAS_GAME_PIECE);

        // check if the release button has been pressed
        if (subsystem.releaseButtonPressed) {
          subsystem.setState(State.L1_PREPARE_TO_SCORE);
          subsystem.releaseButtonPressed = false;
        }
        // check if the game piece is no longer detected by the manipulator; this could occur if
        // it has dropped or knocked out; we don't want to be stuck in this state
        else if (!subsystem.isManipulatorIRBlocked()) {
          subsystem.setState(State.WAITING_FOR_L1_CORAL_IN_FUNNEL);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
  
    CORAL_IN_MANIPULATOR_L2_4 {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setLeftCoralMotorVoltage(
            Volts.of(subsystem.leftCoralMotorCollectionVoltage.get()));
        subsystem.setRightCoralMotorVoltage(
            Volts.of(subsystem.rightCoralMotorCollectionVoltage.get()));
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.HAS_GAME_PIECE);

        // check if the release button has been pressed
        if (subsystem.releaseButtonPressed) {
          subsystem.setState(State.L1_PREPARE_TO_SCORE);
          subsystem.releaseButtonPressed = false;
        }
        // check if the game piece is no longer detected by the manipulator; this could occur if
        // it has dropped or knocked out; we don't want to be stuck in this state
        else if (!subsystem.isManipulatorIRBlocked()) {
          subsystem.setState(State.WAITING_FOR_L2_4_CORAL_IN_FUNNEL);
        }
      }

      L1_PREPARE_TO_SCORE {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setManipulatorMotorVoltage(Volts.of(0.0));
          // move "wrist" to scoring position
          //subsystem.setAlgaeMotorPosition(DEGREES.of(0.0)); // FIXME: set to appropriate scoring angle with a variable
        }

        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.HAS_GAME_PIECE);

          // check if the release button has been pressed
          if (subsystem.releaseButtonPressed) {
            subsystem.setState(State.RELEASE_GAME_PIECE);
            subsystem.releaseButtonPressed = false;
          }
          // check if the game piece is no longer detected by the manipulator; this could occur if
          // it has dropped or knocked out; we don't want to be stuck in this state
          else if (!subsystem.isManipulatorIRBlocked()) {
            subsystem.setState(State.WAITING_FOR_L1_CORAL_IN_FUNNEL);
          }
        }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    L2_4_PREPARE_TO_SCORE {
        @Override
        void onEnter(Manipulator subsystem) {
          subsystem.setManipulatorMotorVoltage(Volts.of(0.0));
          // move "wrist" to scoring position
          //subsystem.setAlgaeMotorPosition(DEGREES.of(0.0)); // FIXME: set to appropriate scoring angle with a variable
        }

        @Override
        void execute(Manipulator subsystem) {
          LEDs.getInstance().requestState(States.HAS_GAME_PIECE);

          // check if the release button has been pressed
          if (subsystem.releaseButtonPressed) {
            subsystem.setState(State.RELEASE_GAME_PIECE);
            subsystem.releaseButtonPressed = false;
          }
          // check if the game piece is no longer detected by the manipulator; this could occur if
          // it has dropped or knocked out; we don't want to be stuck in this state
          else if (!subsystem.isManipulatorIRBlocked()) {
            subsystem.setState(State.WAITING_FOR_L2_4_CORAL_IN_FUNNEL);
          }
        }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    RELEASE_GAME_PIECE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(Volts.of(subsystem.manipulatorReleaseVoltage.get()));
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.RELEASING_GAME_PIECE);

        // wait until the game piece is no longer detected by the manipulator before transitioning
        // back to the waiting for game piece state
        if (!subsystem.isManipulatorIRBlocked()) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    UNINITIALIZED {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(Volts.of(0.0));
      }

      @Override
      void execute(Manipulator subsystem) {
        subsystem.setState(
            State.WAITING_FOR_GAME_PIECE); // default state to WAITING_FOR_CORAL_IN_FUNNEL state
      }

      @Override
      void onExit(Manipulator subsystem) {}
    };

    abstract void execute(Manipulator subsystem);

    abstract void onEnter(Manipulator subsystem);

    abstract void onExit(Manipulator subsystem);
  }

  @Override
  public void periodic() {
    // the first step in periodic is to update the inputs from the IO implementation.
    io.updateInputs(inputs);

    // the next step is to log the inputs to the AdvantageKit logger.
    Logger.processInputs("Manipulator", inputs);

    // Subsystems may need to log additional information that is not part of the inputs. This is
    // done for convenience as additional values can always be logged when replaying a log file.
    // Logging the state is very useful.
    Logger.recordOutput(SUBSYSTEM_NAME + "/State", this.state);

    // If a filter is used, it must be updated every periodic call.
    currentInAmps.calculate(inputs.manipulatorStatorCurrentAmps);

    // If the testing mode is enabled, apply the specified voltage (if not zero). Only run the state
    // machine if testing mode is not enabled. Otherwise, the state machine will "fight" the
    // specified testing value. Similarly, if testing the mechanism using Phoenix Tuner, enable
    // testing mode to ensure that the state machine won't "fight" Phoenix Tuner.
    if (testingMode.get() == 1) {
      if (manipulatorMotorVoltage.get() != 0) {
        //setManipulatorMotorVoltage(Volts.of(manipulatorMotorVoltage.get()));
        setLeftCoralMotorVoltage(Volts.of(leftCoralMotorCollectionVoltage.get()));
        setRightCoralMotorVoltage(Volts.of(rightCoralMotorCollectionVoltage.get()));
        setAlgaeMotorVoltage(Volts.of(algaeMotorCollectionVoltage.get()));
      }
    } else {
      runStateMachine();
    }

    // Log how long this subsystem takes to execute its periodic method.
    // This is useful for debugging performance issues.
    LoggedTracer.record("Manipulator");
  }

  public void resetStateMachine() {
    this.state = State.WAITING_FOR_GAME_PIECE;
  }

  public void releaseGamePiece() {
    releaseButtonPressed = true;
  }

  public void ejectGamePiece(){
    ejectButtonPressed = true;
    //  FIXME: implement eject logic
  }

  public boolean isIndexingCoralL1() {
    return state == State.CENTERING_CORAL_IN_MANIPULATOR_L1;
  }

  public boolean isIndexingCoralL2_L4(){
    return state == State. ; //FIXME: replace with the state for indexing coral l2-l4
  }

  public boolean isIndexingAlgae(){
    return state == State. //FIXME: replace with necessary state
  }

  public boolean hasIndexedCoralL1() { 
    return state == State.GAME_PIECE_IN_MANIPULATOR;
  }

  public boolean hasIndexedCoralL2_L4(){
    return state == State. //FIXME: replace with necessary state
  }

  public boolean hasIndexedAlgae(){
    return state == State. //FIXME: replace with necessary state
  }

  private void setState(State state) {
    this.state = state;
  }

  // Method to switch through the onEnter(), execute(), and onExit() methods of the state machine
  private void runStateMachine() {
    if (state != lastState) {
      lastState.onExit(this);
      lastState = state;
      state.onEnter(this);
    }

    state.execute(this);
  }

  // Methods to the set the voltages of the motors; these methods will be called in the state machine states
  private void setLeftCoralMotorVoltage(Voltage volts){
    io.setLeftCoralMotorVoltage(volts);
  }

  private void setRightCoralMotorVoltage(Voltage volts){
    io.setRightCoralMotorVoltage(volts);
  }

  private void setAlgaeMotorVoltage(Voltage volts){
    io.setAlgaeMotorVoltage(volts);
  }

  // Method to change the angle of the "wrist postion" motor
  public void setAlgaeMotorPosition(Angle deg){ 
    io.setAlgaeAngle(deg); 
  }

  // The inputs class contains the state of the primary and secondary IR sensors. It is useful to
  // have both logged when checking for sensor reliability across matches. Which sensors are used
  // are determined based on the dashboard button.
  private boolean isManipulatorIRBlocked() {
    if (OISelector.getOperatorInterface().getEnablePrimaryIRSensorsTrigger().getAsBoolean()) {
      return inputs.isManipulatorPrimaryIRBlocked;
    } else {
      return inputs.isManipulatorSecondaryIRBlocked;
    }
  }

  private boolean isManipulatorFrontLeftBlocked(){
    return inputs.isManipulatorFrontLeftBlocked;
  }

  private boolean isManipulatorFrontCenterBlocked(){
    return inputs.isManipulatorFrontCenterBlocked;
  }

  private boolean isManipulatorFrontRightBlocked(){
    return inputs.isManipulatorFrontRightBlocked;
  }

  private boolean isManipulatorBackCenterBlocked(){
    return inputs.isManipulatorBackCenterBlocked;
  }

  private boolean isManipulatorBackLeftBlocked(){ // algae sensor
    return inputs.isManipulatorBackLeftBlocked;
  }

  private boolean is

  // A subsystem's system check command is used to verify the functionality of the subsystem. It
  // should perform a sequence of commands (usually encapsulated in another method). The command
  // should always be decorated with an `until` condition that checks for faults in the subsystem
  // and an `andThen` condition that sets the subsystem to a safe state. This ensures that if any
  // faults are detected, the test will stop and the subsystem is always left in a safe state.
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> io.setManipulatorVoltage(Volts.of(3.6))),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.manipulatorVelocityRPS < 2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Manipulator motor not moving as fast as expected",
                            false);
                  }
                }),
            Commands.runOnce(() -> io.setManipulatorVoltage(Volts.of(-2.4))),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.manipulatorVelocityRPS > -2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Manipulator motor moving too slow or in the wrong direction",
                            false);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> io.setManipulatorVoltage(Volts.of(0.0))));
  }
}