package frc.robot.commands.swervedrive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;

public class RunClimb extends Command {

    public DigitalInput LeadClimbLimit = new DigitalInput(ClimbConstants.leadLimit);
    public DigitalInput FollowClimbLimit = new DigitalInput(ClimbConstants.followLimit);

    public CANSparkMax m_lead = new CANSparkMax(ClimbConstants.leadMotor, MotorType.kBrushless);
    public CANSparkMax m_follow = new CANSparkMax(ClimbConstants.followMotor, MotorType.kBrushless);

    boolean finished = false;

    @Override
    public void execute() {
        finished = false;
        while(LeadClimbLimit.get()){
            m_lead.set(0.5);
            m_follow.set(0.5);
        }
        m_follow.set(0);
        m_lead.set(0);
        finished = true;
    }

    @Override
    public boolean isFinished(){
        return finished;
    }

}
