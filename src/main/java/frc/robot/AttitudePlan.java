package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

class AttitudePlan {
    /** Represents one stage of the plan */
    static class State {
        /** direction to face (possibly relative to some external landmark) */
        Rotation2d angle;
        /** change in {@code angle} in radians/ sec */
        double rotSpeed;
        /** @param currentAngle sets the {@code angle} member
         * @param radiansPerSec sets the {@code rotSpeed} member
         */
        State (Rotation2d currentAngle,  double radiansPerSec) {
            angle = currentAngle;
            rotSpeed = radiansPerSec;
        }
    }
}
