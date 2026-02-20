package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

final class OurTrajectories {
    static TrajectoryConfig config = new TrajectoryConfig(3, 2);
    static Trajectory circleTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            Pose2d.kZero, 
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, 0),
                new Translation2d(1, -1)), 
            Pose2d.kZero, 
            config);
}
