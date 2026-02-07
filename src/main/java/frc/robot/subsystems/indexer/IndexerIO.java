package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public Angle indexerPositionRads = Rotations.of(Double.NaN);
    public AngularVelocity indexerVelocityRadPerSec = RPM.of(Double.NaN);
    public Voltage indexerAppliedVolts = Volts.of(Double.NaN);
    public Current indexerCurrentAmps = Amps.of(Double.NaN);
    public boolean indexerConnected = false;
    public Temperature indexerTemperatureC = Celsius.of(Double.NaN);
    public Distance laserCanDistanceM = Millimeters.of(Double.NaN);
  }

  /**
   * Updates the set of loggable inputs for the arm subsystem. This function updates the following
   * inputs:
   *
   * <ul>
   *   <li>{@code armConnected}: Whether the arm motor is connected
   *   <li>{@code armPosition}: The position of the arm motor in radians
   *   <li>{@code armVelocity}: The velocity of the arm motor in radians per second
   *   <li>{@code armAppliedVolts}: The voltage applied to the arm motor in volts
   *   <li>{@code armCurrent}: The current drawn by the arm motor in amps
   * </ul>
   */
  public void updateInputs(IndexerIOInputs inputs) {}

  public void setIndexerVelocity(AngularVelocity velocity) {}

  public void indexerOpenLoop(Voltage voltage) {}
}