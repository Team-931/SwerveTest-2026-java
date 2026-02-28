package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

class AttitudePlan {
    State report(double time) {
        return State.kZero;
    }
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
        static State kZero = new State(Rotation2d.kZero, 0);
    }

    //TODO: members 
}
