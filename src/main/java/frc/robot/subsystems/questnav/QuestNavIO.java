package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;;

public class QuestNavIO {
  protected final QuestNav nav;
  protected final Transform3d robotToNav;

  @AutoLog
  public static class QuestNavIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
  }

  public static record PoseObservation(
      double timestamp,
      Pose3d pose) {
  }


  public QuestNavIO(Transform3d robotToNav) {
    nav = new QuestNav();
    this.robotToNav = robotToNav;
  }

  public void updateInputs(QuestNavIOInputs inputs) {
    inputs.connected = nav.isConnected();

    nav.commandPeriodic();

    List<PoseObservation> poseObservations = new LinkedList<>();

    PoseFrame[] questFrames = nav.getAllUnreadPoseFrames();

    for (PoseFrame frame : questFrames) {
      if (frame.isTracking()) {
        Pose3d pose = frame.questPose3d();
        double timestamp = frame.dataTimestamp();

        Pose3d robotPose = pose.transformBy(robotToNav.inverse());

        poseObservations.add(new PoseObservation(timestamp, robotPose));
      }
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
  }
}
