package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.sim.SimMechanism;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSimConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim extends IntakeIOSpark implements SimMechanism {

  FlywheelSim intakePhysicsSim;

  SparkMaxSim motorSim;
  SparkRelativeEncoderSim encoderSim;

  int intakeNum;

  public IntakeIOSim(int intakeNum) {
    super(intakeNum);

    this.intakeNum = intakeNum;

    DCMotor gearbox = DCMotor.getNeoVortex(1);

    intakePhysicsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1),
                IntakeSimConstants.INTAKE_MOI.in(KilogramSquareMeters),
                IntakeConstants.GEARING),
            gearbox);
    motorSim = new SparkMaxSim(intakeMotor, gearbox);
    encoderSim = new SparkRelativeEncoderSim(intakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakePhysicsSim.setInputVoltage(
        motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    intakePhysicsSim.update(0.02);

    motorSim.iterate(
        intakePhysicsSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

    encoderSim.iterate(intakePhysicsSim.getAngularVelocityRPM(), 0.02);

    encoderSim.setVelocity(intakePhysicsSim.getAngularVelocityRPM());

    Logger.recordOutput(
        "Intake" + intakeNum + "/IntakeVelocity", intakePhysicsSim.getAngularVelocityRPM());
    Logger.recordOutput(
        "Intake" + intakeNum + "/IntakeAppliedVolts", intakePhysicsSim.getInputVoltage());
    Logger.recordOutput(
        "Intake" + intakeNum + "/IntakeCurrentAmps", intakePhysicsSim.getCurrentDrawAmps());
    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {intakePhysicsSim.getCurrentDrawAmps()};
  }
}