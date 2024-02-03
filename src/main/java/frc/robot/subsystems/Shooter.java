package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    CANSparkMax leader = new CANSparkMax(Constants.Shooter.LeaderID, MotorType.kBrushless);
    CANSparkMax follower = new CANSparkMax(Constants.Shooter.FollowerID, MotorType.kBrushless);

    public Shooter(){
        follower.follow(leader);
    }

    public void RunShooter(double speed) {
        leader.set(-speed);
    }
}
