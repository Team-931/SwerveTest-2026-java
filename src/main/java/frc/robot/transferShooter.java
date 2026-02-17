package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class transferShooter {
    TalonFX shooterLeft=new TalonFX(1), shooterMid=new TalonFX(2), shooterRight=new TalonFX(3), 
    transfer=new TalonFX(4);
    Follower followLeft = new Follower(1, MotorAlignmentValue.Aligned);
    {shooterMid.setControl(followLeft); 
        shooterRight.setControl(followLeft);}
//TODO orientation & prefomance activities
void shoot(){shooterLeft.set(0.5);}

}
