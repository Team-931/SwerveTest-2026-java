package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.Constants.DrvConst;

class TrajectoryTranslator {
	Translation2d calculate(Pose2d reportOdometry, State sample) {
        var x = new Translation2d(sample.velocityMetersPerSecond, sample.poseMeters.getRotation())
            .plus((sample.poseMeters.getTranslation() . minus(reportOdometry.getTranslation())) . times(DrvConst.traj_kP));
		return x;
	}
}
