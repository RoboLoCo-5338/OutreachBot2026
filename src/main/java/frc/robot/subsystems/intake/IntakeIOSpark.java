package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

// import au.grapplerobotics.LaserCan;
// import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class IntakeIOSpark extends IntakeIO {

  private final RelativeEncoder intakeEncoder;

  private final Debouncer intake1ConnectedDebouncer = new Debouncer(0.5);
  private final SparkClosedLoopController intakeClosedLoopController;
  private final SparkUtil sparkUtil = new SparkUtil();

  SparkMax intakeMotor;
  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          IntakeConstants.INTAKE_MOTOR_KS, IntakeConstants.INTAKE_MOTOR_KV);
  // LaserCan laserCan = new LaserCan(IntakeConstants.LASERCAN_ID);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake kP", IntakeConstants.INTAKE_MOTOR_VELOCITY_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake kI", IntakeConstants.INTAKE_MOTOR_VELOCITY_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake kD", IntakeConstants.INTAKE_MOTOR_VELOCITY_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake kV", IntakeConstants.INTAKE_MOTOR_KV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Intake kS", IntakeConstants.INTAKE_MOTOR_KS);
  public IntakeIOSpark(int intakeNum) {
    intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    sparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                getIntakeConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    intakeClosedLoopController = intakeMotor.getClosedLoopController();

    // try {
    //   laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    //   laserCan.setRegionOfInterest(
    //       new LaserCan.RegionOfInterest(2, 2, 2, 2)); // TODO: needs to be changed
    //   laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    // } catch (Exception e) {
    //   System.out.println("Error: " + e);
    // }
  }

  /**
   * Gets the configuration used for the Talon FX motor controllers of the shooter subsystem.
   *
   * <p>This method returns a Talon FX configuration with the following settings:
   *
   * <ul>
   *   <li>Neutral mode: Brake
   *   <li>Gravity type: Shooter cosine
   *   <li>Feedback device: Integrated sensor
   *   <li>kP: {@link ShooterConstants#SHOOTER_MOTOR_kP}
   *   <li>kI: {@link ShooterConstants#SHOOTER_MOTOR_kI}
   *   <li>kD: {@link ShooterConstants#SHOOTER_MOTOR_kD}
   *   <li>kV: {@link ShooterConstants#SHOOTER_MOTOR_kV}
   *   <li>kV: {@link ShooterConstants#SHOOTER_MOTOR_kS}
   *   <li>Current limit: 60A (CHANGE THIS VALUE OTHERWISE TORQUE MAY BE LIMITED/TOO HIGH)
   * </ul>
   *
   * <p>These values may need to be changed based on the actual robot hardware and the desired
   * behavior of the elevator.
   *
   * @return the configuration used for the Talon FX motor controllers of the shooter subsystem
   */
  public SparkMaxConfig getIntakeConfig() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
    intakeConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit((int) (IntakeConstants.INTAKE_MOTOR_CURRENT_LIMIT.in(Amps)))
        .voltageCompensation(12.0);

    intakeConfig.absoluteEncoder.velocityConversionFactor(IntakeConstants.GEARING);
    intakeConfig.absoluteEncoder.positionConversionFactor(IntakeConstants.GEARING);
    intakeConfig.idleMode(IdleMode.kCoast);

    return intakeConfig;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sparkUtil.sparkStickyFault = false;

    sparkUtil.ifOk(
        intakeMotor,
        intakeEncoder::getPosition,
        (value) -> inputs.intakePositionRads = Rotations.of(value));
    sparkUtil.ifOk(
        intakeMotor,
        intakeEncoder::getVelocity,
        (value) -> inputs.intakeVelocityRadPerSec = RPM.of(value));
    sparkUtil.ifOk(
        intakeMotor,
        new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
        (values) -> inputs.intakeAppliedVolts = Volts.of(values[0] * values[1]));
    sparkUtil.ifOk(
        intakeMotor,
        intakeMotor::getOutputCurrent,
        (value) -> inputs.intakeCurrentAmps = Amps.of(value));
    sparkUtil.ifOk(
        intakeMotor,
        intakeMotor::getMotorTemperature,
        (value) -> inputs.intakeTemperatureC = Celsius.of(value));

    LoggedTunableNumber.ifChanged(
        1,
        () ->
            intakeMotor.configure(
                getIntakeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        5, () -> feedforward = new SimpleMotorFeedforward(kS.get(), kV.get()), kV, kS);

    inputs.intakeConnected = intake1ConnectedDebouncer.calculate(!sparkUtil.sparkStickyFault);
    // Measurement m1 = laserCan.getMeasurement();
    // if (m1 == null) {
    //   inputs.laserCanDistanceM = Millimeters.of(-1);
    // } else if (m1.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    //   inputs.laserCanDistanceM = Millimeters.of(m1.distance_mm);
    // } else if (m1.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
    //   inputs.laserCanDistanceM = Millimeters.of(-1);
    // }
  }

  @Override
  public void intakeOpenLoop(Voltage voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeVelocity(AngularVelocity velocity) {
    intakeClosedLoopController.setReference(
        velocity.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(velocity.in(RPM)));
  }

  public void follow(IntakeIOSpark leader, boolean inverted) {
    sparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                getIntakeConfig().follow(leader.intakeMotor, inverted),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }
}