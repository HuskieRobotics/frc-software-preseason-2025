package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public interface elevatorIO(){
  
  public static class ElevatorIOInputs {
  boolean connectedLead = false;
  boolean connectedFollower = false;
  //Voltage supply - volts
  double voltageSupplyLead = 0.0;
  double voltageSupplyFollower = 0.0;
  //Stator Current - amps
  double statorCurrentAmpsLead = 0.0;
  double statorCurrentAmpsFollower = 0.0;
  //Supply Current - amps
  double supplyCurrentAmpsLead = 0.0;
  double supplyCurrentAmpsFollower = 0.0;
  //Position - inches
  double positionInches = 0.0;
  //Position - rotations
  double positionRotations = 0.0;
  //Velocity - rotations per second
  double velocityRPSLead = 0.0;
  //Closed loop error - rotations
  double closedLoopError = 0.0;
  //Closed loop Reference - rotations
  double closedLoopReference = 0.0;
  //Temperature - degrees celsius
  double temperatureCelsiusLead = 0.0;
  double temperatureCelsiusFollower = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
  public default void setElevatorVoltage(double voltage) {}
  public default void setPosition(Distance position) {}

  public default void zeroPosition() {}
}



