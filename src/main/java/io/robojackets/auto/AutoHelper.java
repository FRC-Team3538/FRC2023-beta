package io.robojackets.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import java.util.List;

public class AutoHelper {
  public static Trajectory LoadTrajectory(String name, TrajectoryConfig config) {
    return new Trajectory();
  }

  public static List<Trajectory> LoadTrajectorySplit(String name, TrajectoryConfig config) {
    return List.of();
  }
}
