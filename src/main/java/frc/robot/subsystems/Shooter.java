package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

//for dictionary
   
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;

public class Shooter extends SubsystemBase {

    TalonFX m_leader = new TalonFX(ShooterConstants.m_LeaderID);
    TalonFX m_Follower = new TalonFX(ShooterConstants.m_FollowerID);

    Follower follow;
    //Key: Distance
    //Value: Angle
    Dictionary<Double, Double> dictionary = new Hashtable<>();
    public Shooter() {
        follow = new Follower(ShooterConstants.m_LeaderID, true);

       // dictionary.put(null, null);
    }

    public void RunShooter(double speed) {

        VoltageOut leaderSpeed = new VoltageOut(14 * speed);
        m_leader.setControl(leaderSpeed);
        m_Follower.setControl(follow);

    }
    public Command StartShooter(){
        return new RunCommand(() -> RunShooter(1), this);
    }
      public Command StopShooter(){
        return new RunCommand(() -> RunShooter(0), this);
    }








}
