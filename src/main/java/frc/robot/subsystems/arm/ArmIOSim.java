package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.sim.SimMechanism;
import frc.robot.subsystems.arm.ArmConstants.ArmSimConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim extends ArmIOSpark implements SimMechanism {

  FlywheelSim armPhysicsSim;

  SparkMaxSim motorSim;
  SparkRelativeEncoderSim encoderSim;

  int armNum;

  public ArmIOSim(int armNum) {
    super(armNum);

    this.armNum = armNum;

    DCMotor gearbox = DCMotor.getNeoVortex(1);

    armPhysicsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1),
                ArmSimConstants.ARM_MOI.in(KilogramSquareMeters),
                ArmConstants.GEARING),
            gearbox);
    motorSim = new SparkMaxSim(armMotor, gearbox);
    encoderSim = new SparkRelativeEncoderSim(armMotor);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armPhysicsSim.setInputVoltage(
        motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    armPhysicsSim.update(0.02);

    motorSim.iterate(
        armPhysicsSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

    encoderSim.iterate(armPhysicsSim.getAngularVelocityRPM(), 0.02);

    encoderSim.setVelocity(armPhysicsSim.getAngularVelocityRPM());
    Logger.recordOutput("Arm" + armNum + "/ArmVelocity", armPhysicsSim.getAngularVelocityRPM());
    Logger.recordOutput("Arm" + armNum + "/ArmAppliedVolts", armPhysicsSim.getInputVoltage());
    Logger.recordOutput("Arm" + armNum + "/ArmCurrentAmps", armPhysicsSim.getCurrentDrawAmps());

    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {armPhysicsSim.getCurrentDrawAmps()};
  }
}
