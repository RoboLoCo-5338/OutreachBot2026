package frc.robot.subsystems.indexer;

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

public class IndexerIOSpark extends IndexerIO {

  private final RelativeEncoder indexerEncoder;

  private final Debouncer indexer1ConnectedDebouncer = new Debouncer(0.5);
  private final SparkClosedLoopController indexerClosedLoopController;
  private final SparkUtil sparkUtil = new SparkUtil();

  SparkMax indexerMotor;
  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          IndexerConstants.INDEXER_MOTOR_KS, IndexerConstants.INDEXER_MOTOR_KV);
  // LaserCan laserCan = new LaserCan(IndexerConstants.LASERCAN_ID);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Indexer kP", IndexerConstants.INDEXER_MOTOR_VELOCITY_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Indexer kI", IndexerConstants.INDEXER_MOTOR_VELOCITY_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Indexer kD", IndexerConstants.INDEXER_MOTOR_VELOCITY_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Indexer kV", IndexerConstants.INDEXER_MOTOR_KV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Indexer kS", IndexerConstants.INDEXER_MOTOR_KS);
  public IndexerIOSpark(int indexerNum) {
    indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    indexerEncoder = indexerMotor.getEncoder();

    sparkUtil.tryUntilOk(
        indexerMotor,
        5,
        () ->
            indexerMotor.configure(
                getIndexerConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    indexerClosedLoopController = indexerMotor.getClosedLoopController();

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
  public SparkMaxConfig getIndexerConfig() {
    SparkMaxConfig indexerConfig = new SparkMaxConfig();

    indexerConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
    indexerConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit((int) (IndexerConstants.INDEXER_MOTOR_CURRENT_LIMIT.in(Amps)))
        .voltageCompensation(12.0);

    indexerConfig.absoluteEncoder.velocityConversionFactor(IndexerConstants.GEARING);
    indexerConfig.absoluteEncoder.positionConversionFactor(IndexerConstants.GEARING);
    indexerConfig.idleMode(IdleMode.kCoast);

    return indexerConfig;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    sparkUtil.sparkStickyFault = false;

    sparkUtil.ifOk(
        indexerMotor,
        indexerEncoder::getPosition,
        (value) -> inputs.indexerPositionRads = Rotations.of(value));
    sparkUtil.ifOk(
        indexerMotor,
        indexerEncoder::getVelocity,
        (value) -> inputs.indexerVelocityRadPerSec = RPM.of(value));
    sparkUtil.ifOk(
        indexerMotor,
        new DoubleSupplier[] {indexerMotor::getAppliedOutput, indexerMotor::getBusVoltage},
        (values) -> inputs.indexerAppliedVolts = Volts.of(values[0] * values[1]));
    sparkUtil.ifOk(
        indexerMotor,
        indexerMotor::getOutputCurrent,
        (value) -> inputs.indexerCurrentAmps = Amps.of(value));
    sparkUtil.ifOk(
        indexerMotor,
        indexerMotor::getMotorTemperature,
        (value) -> inputs.indexerTemperatureC = Celsius.of(value));

    LoggedTunableNumber.ifChanged(
        1,
        () ->
            indexerMotor.configure(
                getIndexerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        5, () -> feedforward = new SimpleMotorFeedforward(kS.get(), kV.get()), kV, kS);

    inputs.indexerConnected = indexer1ConnectedDebouncer.calculate(!sparkUtil.sparkStickyFault);
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
  public void indexerOpenLoop(Voltage voltage) {
    indexerMotor.setVoltage(voltage);
  }

  @Override
  public void setIndexerVelocity(AngularVelocity velocity) {
    indexerClosedLoopController.setReference(
        velocity.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(velocity.in(RPM)));
  }

  public void follow(IndexerIOSpark leader, boolean inverted) {
    sparkUtil.tryUntilOk(
        indexerMotor,
        5,
        () ->
            indexerMotor.configure(
                getIndexerConfig().follow(leader.indexerMotor, inverted),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }
}