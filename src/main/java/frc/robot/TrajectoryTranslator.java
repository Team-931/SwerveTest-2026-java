package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.Constants.DrvConst;

class TrajectoryTranslator {
/** Given a current location and a desired State, choose outputs to achieve the state. 
 * @param reportOdometry where we think we are, and which way we face
 * @param sample {@link Trajectory.State} to try to achieve
 * @return speed vector to drive
*/
// TODO: also calculate for direction to face.
	Translation2d calculate(Pose2d reportOdometry, State sample) {
        var x = new Translation2d(sample.velocityMetersPerSecond, sample.poseMeters.getRotation())
            .plus((sample.poseMeters.getTranslation() . minus(reportOdometry.getTranslation())) . times(DrvConst.traj_kP));
		return x;
	}
}
