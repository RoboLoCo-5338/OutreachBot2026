package frc.robot.subsystems.questnav;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class QuestNav extends SubsystemBase {
  private final QuestNavConsumer consumer;
  private final QuestNavIO io;
  private final QuestNavIOInputsAutoLogged input;
  private final Alert disconnectedAlert;

  private final Matrix<N3, N1> QuestNavSTDevs = VecBuilder.fill(0.02, 0.02, 0.0872665); //20mm 5 degrees

  public QuestNav(QuestNavConsumer consumer, QuestNavIO io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.input = new QuestNavIOInputsAutoLogged();

    // Initialize disconnected alerts
    this.disconnectedAlert = new Alert(
        "QuestNav is disconnected.", AlertType.kWarning);

  }

  public void resetPose(Pose3d pose) {
    io.nav.setPose(pose);
  }

  @Override
  public void periodic() {
    io.updateInputs(input);

    List<Pose3d> robotPoses = new LinkedList<>();
    List<Pose3d> robotPosesAccepted = new LinkedList<>();
    List<Pose3d> robotPosesRejected = new LinkedList<>();

    disconnectedAlert.set(!input.connected);

    for (var observation : input.poseObservations) {
      // Add pose to log
      robotPoses.add(observation.pose());

      // Check whether to reject pose
      boolean rejectPose = Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

          // Must be within the field boundaries
          || observation.pose().getX() < 0.0
          || observation.pose().getX() > aprilTagLayout.getFieldLength()
          || observation.pose().getY() < 0.0
          || observation.pose().getY() > aprilTagLayout.getFieldWidth();

      if (rejectPose) {
        robotPosesRejected.add(observation.pose());
        continue;
      } else {
        robotPosesAccepted.add(observation.pose());
      }

      consumer.accept(
          observation.pose().toPose2d(), observation.timestamp(), QuestNavSTDevs);

      Logger.recordOutput(
          "QuestNav/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "QuestNav/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "QuestNav/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

    }
  }

  @FunctionalInterface
  public interface QuestNavConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> QuestNavSTDevs);
  }
}
