package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {
  public static final int SHOOTER_MOTOR_1_ID = 11;
  public static final int SHOOTER_MOTOR_2_ID = 12;
  public static final double SHOOTER_MOTOR_VELOCITY_KP = 2.7403E-43; // TODO: TUNE VALUES
  public static final double SHOOTER_MOTOR_VELOCITY_KI = 0; // TODO: TUNE VALUES
  public static final double SHOOTER_MOTOR_VELOCITY_KD = 0; // TODO: TUNE VALUES
  public static final double SHOOTER_MOTOR_KV = 0.020001; // TODO: TUNE VALUES
  public static final double SHOOTER_MOTOR_KS = 0.2058; // TODO: TUNE VALUES
  public static final Current SHOOTER_MOTOR_CURRENT_LIMIT = Amps.of(60);
  public static final double GEARING = 2 * Math.PI;

  public static final AngularVelocity SHOOTER_REVERSE_VELOCITY = RotationsPerSecond.of(-1.0); // TODO: TUNE VALUES
  public static final AngularVelocity SHOOTER_FORWARD_VELOCITY = RotationsPerSecond.of(1.0); // TODO: TUNE VALUES
  public static final AngularVelocity SHOOTER_NO_VELOCITY = RotationsPerSecond.of(0.0);

  public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05); // TODO: TUNE VALUES

  public static final class ShooterSimConstants {
    public static final MomentOfInertia SHOOTER_MOI = PoundSquareInches.of(0.729044); // TODO: TUNE VALUES
  }
}