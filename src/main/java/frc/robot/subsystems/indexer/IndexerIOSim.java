package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.sim.SimMechanism;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerSimConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerIOSim extends IndexerIOSpark implements SimMechanism {

  FlywheelSim indexerPhysicsSim;

  SparkMaxSim motorSim;
  SparkRelativeEncoderSim encoderSim;

  int indexerNum;

  public IndexerIOSim(int indexerNum) {
    super(indexerNum);

    this.indexerNum = indexerNum;

    DCMotor gearbox = DCMotor.getNeoVortex(1);

    indexerPhysicsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1),
                IndexerSimConstants.INDEXER_MOI.in(KilogramSquareMeters),
                IndexerConstants.GEARING),
            gearbox);
    motorSim = new SparkMaxSim(indexerMotor, gearbox);
    encoderSim = new SparkRelativeEncoderSim(indexerMotor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerPhysicsSim.setInputVoltage(
        motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    indexerPhysicsSim.update(0.02);

    motorSim.iterate(
        indexerPhysicsSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

    encoderSim.iterate(indexerPhysicsSim.getAngularVelocityRPM(), 0.02);

    encoderSim.setVelocity(indexerPhysicsSim.getAngularVelocityRPM());
    Logger.recordOutput(
        "Indexer" + indexerNum + "/IndexerVelocity", indexerPhysicsSim.getAngularVelocityRPM());
    Logger.recordOutput(
        "Indexer" + indexerNum + "/IndexerAppliedVolts", indexerPhysicsSim.getInputVoltage());
    Logger.recordOutput(
        "Indexer" + indexerNum + "/IndexerCurrentAmps", indexerPhysicsSim.getCurrentDrawAmps());
    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {indexerPhysicsSim.getCurrentDrawAmps()};
  }
}