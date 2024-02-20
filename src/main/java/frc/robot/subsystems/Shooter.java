package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    TalonFX m_leader = new TalonFX(ShooterConstants.m_LeaderID);
    TalonFX m_Follower = new TalonFX(ShooterConstants.m_FollowerID);
    Follower follow;

    public Shooter() {
        follow = new Follower(ShooterConstants.m_LeaderID, true);
    }

    public void RunShooter(double speed) {

        VoltageOut leaderSpeed = new VoltageOut(14 * speed);
        m_leader.setControl(leaderSpeed);
        m_Follower.setControl(follow);

    }

}
