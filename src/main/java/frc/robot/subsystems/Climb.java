package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

    public DigitalInput LeadClimbLimit = new DigitalInput(ClimbConstants.leadLimit);
    public DigitalInput FollowClimbLimit = new DigitalInput(ClimbConstants.followLimit);

    public CANSparkMax m_lead = new CANSparkMax(ClimbConstants.leadMotor, MotorType.kBrushless);
    public CANSparkMax m_follow = new CANSparkMax(ClimbConstants.followMotor, MotorType.kBrushless);


    public Climb(){
    }

    public Command RunClimb(){
        return new RunCommand(() ->
        {
                while(LeadClimbLimit.get()){
                    m_lead.set(0.5);
                    m_follow.set(0.5);
                }
                m_follow.set(0);
                m_lead.set(0);
        }, this);
    }

    public void move(double speed){
        m_follow.set(speed);
        m_lead.set(speed);

    }
    public Command ResetClimb(){
        return new SequentialCommandGroup(
            new RunCommand(() ->
            {
                while(!LeadClimbLimit.get()){
                    m_lead.set(-0.5);
                }
                m_follow.set(0);
            }, this),

            new RunCommand(() ->
            {
                while(!FollowClimbLimit.get()){
                    m_follow.set(-0.5);
                }
                m_follow.set(0);
            }, this),

            new RunCommand(() ->
            {
                while(LeadClimbLimit.get()){
                    m_lead.set(-0.5);
                }
                m_follow.set(0);
            }, this),

            new RunCommand(() ->
            {
                while(FollowClimbLimit.get()){
                    m_follow.set(-0.5);
                }
                m_follow.set(0);
            }, this)
        );
    }
    

    @Override
    public void periodic(){
        if(!LeadClimbLimit.get())
        {
            m_lead.getEncoder().setPosition(0);
        }
        SmartDashboard.putBoolean("Lead Climb Limit Switch", LeadClimbLimit.get());
        SmartDashboard.putBoolean("Follow Climb Limit Switch", FollowClimbLimit.get());
    
        SmartDashboard.putNumber("Lead Climb ENC", m_lead.getEncoder().getPosition());
    }

}
