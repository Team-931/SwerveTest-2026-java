package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShootConstants;


public class transferShooter { Servo leftServo=new Servo(1), rightServo=new Servo(9);
    TalonFX shooterLeft=new TalonFX(1), shooterMid=new TalonFX(2), shooterRight=new TalonFX(3), 
    transfer=new TalonFX(4);
    Follower followLeft = new Follower(1, MotorAlignmentValue.Aligned);
    {shooterMid.setControl(followLeft); 
        shooterRight.setControl(followLeft);}
//TODO orientation & prefomance activities
void shoot(boolean on){shooterLeft.set (on?ShootConstants.launch_speed:0);}
void setTransfer(boolean on) {
    transfer.set(on ? ShootConstants.transferPower : 0);
}
//TODO check if the requested position is too far
void adjustHood(double out) {
    rightServo.setPosition(out); leftServo.setPosition(out);
}
}
