package frc.robot.subsystems.arm;

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

public class Arm extends SubsystemBase implements SysIdSubsystem {

  public final ArmIO io1, io2;
  public final ArmIOInputsAutoLogged inputs1 = new ArmIOInputsAutoLogged(),
      inputs2 = new ArmIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine1;
  private final SysIdRoutine sysIdRoutine2;

  private final Alert ArmDisconnectedAlert =
      new Alert("Arm motor disconnected", AlertType.kWarning);

  public Arm(ArmIO io1, ArmIO io2) {
    super();
    this.io1 = io1;
    this.io2 = io2;
    this.sysIdRoutine1 =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new Mechanism(io1::armOpenLoop, null, this));
    this.sysIdRoutine2 =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new Mechanism(io2::armOpenLoop, null, this));
  }

@Override
  public void periodic() {
    io1.updateInputs(inputs1);
    Logger.processInputs("Arm1", inputs1);

    ArmDisconnectedAlert.set(!inputs1.armConnected && Constants.CURRENT_MODE != Mode.SIM);

    io2.updateInputs(inputs2);
    Logger.processInputs("Arm2", inputs2);

    ArmDisconnectedAlert.set(!inputs2.armConnected && Constants.CURRENT_MODE != Mode.SIM);
  }

  /**
   * Sets the shooter to the given velocity in degrees per second.
   *
   * <p>This is a non-blocking call and will not wait until the shooter is at the requested
   * velocity.
   *
   * @param velocity The velocity to set the arm to in degrees per second.
   * @return A command that sets the arm to the given velocity.
   */
  public Command setArmVelocity(Supplier<AngularVelocity> velocity) {
    return new InstantCommand(
            () -> {
              io1.setArmVelocity(velocity.get());
              io2.setArmVelocity(velocity.get());
            },
            this)
        .withName("Set Arm Velocity");
  }

  public Command setArmVelocity(
      Supplier<AngularVelocity> velocity1, Supplier<AngularVelocity> velocity2) {
    return new InstantCommand(
            () -> {
              io1.setArmVelocity(velocity1.get());
              io2.setArmVelocity(velocity2.get());
            },
            this)
        .withName("Set Arm Velocity(Dual)");
  }

  public Command reset(Direction direction) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  io1.armOpenLoop(Volts.of(0));
                  io2.armOpenLoop(Volts.of(0));
                },
                this),
            Commands.idle(this)
                .until(
                    () ->
                        inputs1.armVelocityRadPerSec.isNear(
                                RotationsPerSecond.ofBaseUnits(0), ArmConstants.RESET_TOLERANCE)
                            && inputs2.armVelocityRadPerSec.isNear(
                                RotationsPerSecond.ofBaseUnits(0),
                                ArmConstants.RESET_TOLERANCE)),
            Commands.runOnce(
                () -> {
                  io1.armOpenLoop(Volts.of(0));
                  io2.armOpenLoop(Volts.of(0));
                },
                this))
        .withName("Reset Arm");
  }

  @Override
  public List<SysIdTarget> getSysIdTargets() {
    return List.of(
        new SysIdTarget("Arm1 ", sysIdRoutine1, this::reset),
        new SysIdTarget("Arm2 ", sysIdRoutine2, this::reset));
  }
}