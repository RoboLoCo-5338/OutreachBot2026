package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIdSubsystem;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase implements SysIdSubsystem.SysIdSingleSubsystem {
  public final IndexerIO io;
  public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  private final Alert IndexerDisconnectedAlert =
      new Alert("Indexer motor disconnected", AlertType.kWarning);

  public Indexer(IndexerIO io) {
    super();
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Indexer/SysIdState", state.toString())),
            new Mechanism(io::indexerOpenLoop, null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    IndexerDisconnectedAlert.set(!inputs.indexerConnected && Constants.CURRENT_MODE != Mode.SIM);
  }

  /**
   * Sets the intake to the given velocity in degrees per second.
   *
   * <p>This is a non-blocking call and will not wait until the intake is at the requested
   * velocity.
   *
   * @param velocity The velocity to set the intake to in degrees per second.
   * @return A command that sets the intake to the given velocity.
   */
  public Command setIndexerVelocity(Supplier<AngularVelocity> velocity) {
    return new InstantCommand(
            () -> {
              io.setIndexerVelocity(velocity.get());
            },
            this)
        .withName("Set Indexer Velocity");
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  public Command reset(Direction direction) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  io.indexerOpenLoop(Volts.of(0));
                },
                this),
            Commands.idle(this)
                .until(
                    () ->
                        inputs.indexerVelocityRadPerSec.isNear(
                                RotationsPerSecond.ofBaseUnits(0), IndexerConstants.RESET_TOLERANCE)),
            Commands.runOnce(
                () -> {
                  io.indexerOpenLoop(Volts.of(0));
                },
                this))
        .withName("Reset Indexer");
  }
}