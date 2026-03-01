package frc.robot;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

final class Constants {
    static final double deadBand = .05;
    static final double nominalVoltage = 12; /* Volts */
    static final double krakenFreeSpeed = (6000); /* RPM */
  
    static final class DrvConst {
         static final double kMaxSpeed = 3.0, overloadSpeed = kMaxSpeed/* or SwvModConst.freeVeloc */; // 3 meters per second
         static final double kMaxAngularSpeed = 1*Math.PI; // 0.5 rotation per second, was 0.5 rad/s before
         static final class Setup {
            final int driveId, turnId;
            final double absOffset;
            final String name;
            Setup(int drv, int trn, double offset, String nam3) {
                driveId = drv;
                turnId = trn;
                absOffset = offset;
                name = nam3;
            }
         }

         static final double baseOffset = .1875; // .18611111111111111111111111111111
         static Setup frontLeft = new Setup(3, 5, baseOffset + .75, "FL"),
                     frontRight = new Setup(9, 6, baseOffset, "FR"),
                     backLeft = new Setup(2, 4, baseOffset + .5, "BL"),
                     backRight = new Setup(8, 7, baseOffset + .25, "BR");
        
         static final double halfWidth = Units.inchesToMeters(10.75), halfLength = Units.inchesToMeters(11.5);
         static final Translation2d frontLeftLocation = new Translation2d(halfWidth, halfLength); // unit: meters; x is forward dist from center, y is leftward
         static final Translation2d frontRightLocation = new Translation2d(halfWidth, -halfLength);
         static final Translation2d backLeftLocation = new Translation2d(-halfWidth, halfLength);
         static final Translation2d backRightLocation = new Translation2d(-halfWidth, -halfLength);
         
         static final Rotation2d ClockW90 = new Rotation2d(0, 1);
         static final Translation2d frontLeftClW = frontLeftLocation.rotateBy(ClockW90); 
         static final Translation2d frontRightClW = frontRightLocation.rotateBy(ClockW90);
         static final Translation2d backLeftClW = backLeftLocation.rotateBy(ClockW90);
         static final Translation2d backRightClW = backRightLocation.rotateBy(ClockW90);
         
         static final double driveRadius = 
                 MyMath.max(
                    frontLeftLocation.getNorm(), frontRightLocation.getNorm(), 
                    backLeftLocation.getNorm(), backRightLocation.getNorm()
                    );
                 
        /**  how much to correct position errors during a Trajectory:
         * units of proportion / sec
         */
         static final double traj_kP = 2;
         static final double attitudeP = 0;

    }
    static final class SwvModConst {
        final static ClosedLoopSlot posSlot = ClosedLoopSlot.kSlot0;
        final static double posP = .5;
        final static ClosedLoopSlot velSlot = ClosedLoopSlot.kSlot1;
        final static double velP = .0001;
        static final double kWheelRadius = .05931 / 2; /* 0.034 */; //meter //diameter: 2.335 in, 59.31 mm
        static final double freeVeloc = 4.63; //5.31;
         //TODO: tune better
        static final double DrvFF = 1 / freeVeloc; // Officially Volt /(m/s), conjectured: proportional output / (m/s)
        static final int turnGearing = 28, driveGearing = 4;
        static final double driveConversion = 2 * Math.PI * kWheelRadius / driveGearing, // motor rotations to output meters
        // TODO Decide whether turn unit should be radians, or rotations as currently.
                            turnConversion = 1. / turnGearing;
        static final double velI = 0.001, velIZone = .05;
        static final double turnI = 0.003 * posP, turnIZone = 1. / 64;
        static final double minSpd = .001, // mm / s
                            minSpdSq = minSpd*minSpd;
    }
    static final class ShootConstants {
        static final double launch_speed = 0.5;
        static final double transferPower = .5;
        static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
        static final double kMinPosition = 0.01;
        static final double kMaxPosition = 0.77;
        static final double kPositionTolerance = 0.01;
        static final Distance kServoLength = Millimeters.of(100);
        /** Time for hood Servos to run their full length */
        static final double hoodFullLengthTime = kServoLength.div(kMaxServoSpeed).in(Seconds);
        static final int leftShooterID = 1;
        static final int midShootID = 2;
        static final int RightShootID = 3;
        static final int transferMotorID = 4;
        static final int leftServoID = 1;
        static final int rightServoID = 9;
    }
    static final class FeederConst {
    
        static final int motorID = 5;
        static final double runPower = .3;
    }
    
}
