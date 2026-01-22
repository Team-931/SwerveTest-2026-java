package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

public final class Constants {
    public final class Module {
        public final static ClosedLoopSlot posSlot = ClosedLoopSlot.kSlot0;
        public final static double posP = .1;
        public final static ClosedLoopSlot velSlot = ClosedLoopSlot.kSlot1;
        public final static double velP = .0001;
    }
    
}
