package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class IndexerConstants {
  public static final int INDEXER_MOTOR_ID = 16;
  public static final double INDEXER_MOTOR_VELOCITY_KP = 1.0716E-06; // TODO: TUNE VALUES
  public static final double INDEXER_MOTOR_VELOCITY_KI = 0; // TODO: TUNE VALUES
  public static final double INDEXER_MOTOR_VELOCITY_KD = 0; // TODO: TUNE VALUES
  public static final double INDEXER_MOTOR_KV = 0.0084407; // TODO: TUNE VALUES
  public static final double INDEXER_MOTOR_KS = 0.0017394; // TODO: TUNE VALUES
  public static final Current INDEXER_MOTOR_CURRENT_LIMIT = Amps.of(60);
  public static final double GEARING = 1.0;

  public static final AngularVelocity INDEXER_REVERSE_VELOCITY = RotationsPerSecond.of(-1.0); // TODO: TUNE VALUES
  public static final AngularVelocity INDEXER_FORWARD_VELOCITY = RotationsPerSecond.of(1.0); // TODO: TUNE VALUES
  public static final AngularVelocity INDEXER_NO_VELOCITY = RotationsPerSecond.of(0.0);

  public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05); // TODO: TUNE VALUES

  public static final class IndexerSimConstants {
    public static final MomentOfInertia INDEXER_MOI = PoundSquareInches.of(0.729044); // TODO: TUNE VALUES
  }
}