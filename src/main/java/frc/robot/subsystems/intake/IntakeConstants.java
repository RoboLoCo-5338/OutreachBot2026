package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
  public static final int INTAKE_MOTOR_ID = 13;
  public static final double INTAKE_MOTOR_VELOCITY_KP = 1.0716E-06; // TODO: TUNE VALUES
  public static final double INTAKE_MOTOR_VELOCITY_KI = 0; // TODO: TUNE VALUES
  public static final double INTAKE_MOTOR_VELOCITY_KD = 0; // TODO: TUNE VALUES
  public static final double INTAKE_MOTOR_KV = 0.0084407; // TODO: TUNE VALUES
  public static final double INTAKE_MOTOR_KS = 0.0017394; // TODO: TUNE VALUES
  public static final Current INTAKE_MOTOR_CURRENT_LIMIT = Amps.of(60);
  public static final double GEARING = 2 * Math.PI;

  public static final AngularVelocity INTAKE_REVERSE_VELOCITY = RotationsPerSecond.of(-1.0); // TODO: TUNE VALUES
  public static final AngularVelocity INTAKE_FORWARD_VELOCITY = RotationsPerSecond.of(1.0); // TODO: TUNE VALUES
  public static final AngularVelocity INTAKE_NO_VELOCITY = RotationsPerSecond.of(0.0);
  public static final Voltage INTAKE_IN_VOLTAGE = Volts.of(-9.6);
  public static final Voltage INTAKE_OUT_VOLTAGE = Volts.of(7.2);

  public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05); // TODO: TUNE VALUES

  public static final class IntakeSimConstants {
    public static final MomentOfInertia INTAKE_MOI = PoundSquareInches.of(0.729044); // TODO: TUNE VALUES
  }
}