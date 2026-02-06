package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ArmIOSpark extends ArmIO {

  private final RelativeEncoder armEncoder;

  private final Debouncer arm1ConnectedDebouncer = new Debouncer(0.5);

  private final SparkClosedLoopController armClosedLoopController;
  private final SparkUtil sparkUtil = new SparkUtil();

  SparkMax armMotor;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ArmConstants.ARM_MOTOR_KS, ArmConstants.ARM_MOTOR_KV);
  // LaserCan laserCan = new LaserCan(ArmConstants.LASERCAN_ID);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Arm kP", ArmConstants.ARM_MOTOR_VELOCITY_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Arm kI", ArmConstants.ARM_MOTOR_VELOCITY_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Arm kD", ArmConstants.ARM_MOTOR_VELOCITY_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm kV", ArmConstants.ARM_MOTOR_KV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm kS", ArmConstants.ARM_MOTOR_KS);

  public ArmIOSpark(int armNum) {
    armMotor =
        new SparkMax(
            armNum == 1 ? ArmConstants.ARM_MOTOR_1_ID : ArmConstants.ARM_MOTOR_2_ID,
            MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();

    sparkUtil.tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    armClosedLoopController = armMotor.getClosedLoopController();
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
  public SparkMaxConfig getArmConfig() {
    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
    armConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit((int) (ArmConstants.ARM_MOTOR_CURRENT_LIMIT.in(Amps)))
        .voltageCompensation(12.0);

    armConfig.absoluteEncoder.velocityConversionFactor(ArmConstants.GEARING);
    armConfig.absoluteEncoder.positionConversionFactor(ArmConstants.GEARING);
    armConfig.idleMode(IdleMode.kCoast);

    return armConfig;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkUtil.sparkStickyFault = false;

    sparkUtil.ifOk(
        armMotor, armEncoder::getPosition, (value) -> inputs.armPositionRads = Rotations.of(value));
    sparkUtil.ifOk(
        armMotor, armEncoder::getVelocity, (value) -> inputs.armVelocityRadPerSec = RPM.of(value));
    sparkUtil.ifOk(
        armMotor,
        new DoubleSupplier[] {armMotor::getAppliedOutput, armMotor::getBusVoltage},
        (values) -> inputs.armAppliedVolts = Volts.of(values[0] * values[1]));
    sparkUtil.ifOk(
        armMotor, armMotor::getOutputCurrent, (value) -> inputs.armCurrentAmps = Amps.of(value));
    sparkUtil.ifOk(
        armMotor,
        armMotor::getMotorTemperature,
        (value) -> inputs.armTemperatureC = Celsius.of(value));

    LoggedTunableNumber.ifChanged(
        1,
        () ->
            armMotor.configure(
                getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        5, () -> feedforward = new SimpleMotorFeedforward(kS.get(), kV.get()), kV, kS);

    inputs.armConnected = arm1ConnectedDebouncer.calculate(!sparkUtil.sparkStickyFault);
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
  public void armOpenLoop(Voltage voltage) {
    armMotor.setVoltage(voltage);
  }

  @Override
  public void setArmVelocity(AngularVelocity velocity) {
    armClosedLoopController.setReference(
        velocity.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(velocity.in(RPM)));
  }

  public void follow(ArmIOSpark leader, boolean inverted) {
    sparkUtil.tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                getArmConfig().follow(leader.armMotor, inverted),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }
}
