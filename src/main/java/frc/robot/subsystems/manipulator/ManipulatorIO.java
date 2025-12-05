package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  //
  @AutoLog
  public static class ManipulatorIOInputs {

    //This is recording the values of the left, right, and coral motors

    //stator
    double leftCoralMotorStatorCurrentAmps = 0;
    double rightCoralMotorStatorCurrentAmps = 0;
    double algaeMotorStatorCurrentAmps = 0;
    
    //supply
    double leftCoralMotorSupplyCurrentAmps = 0;
    double rightCoralMotorSupplyCurrentAmps = 0;
    double algaeMotorSupplyCurrentAmps = 0;

    //temp
    double leftCoralMotorTemperature = 0;
    double rightCoralMotorTemperature = 0;
    double algaeMotorTemperature = 0;

    //voltage
    double leftCoralMotorVoltage = 0;
    double rightCoralMotorVoltage = 0;
    double algaeMotorVoltage = 0;

    //connection status
    boolean leftCoralMotorConnected = false;
    boolean rightCoralMotorConnected = false;
    boolean algaeMotorConnected = false;


  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setManipulatorVoltage(Voltage volts) {}



  
  
}
