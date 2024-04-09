package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;

public class Climb extends SubsystemBase {

    public DigitalInput LeadClimbLimit = new DigitalInput(ClimbConstants.leadLimit);
    public DigitalInput FollowClimbLimit = new DigitalInput(ClimbConstants.followLimit);

    public CANSparkMax m_lead = new CANSparkMax(ClimbConstants.leadMotor, MotorType.kBrushless);
    public CANSparkMax m_follow = new CANSparkMax(ClimbConstants.followMotor, MotorType.kBrushless);

    int leadForward = -1;
    int followForward = -1;
    boolean lifted1 = false;
    boolean lifted2 = false;
    boolean setClimb1 = false;
    boolean setClimb2 = false;
    boolean reversed1 = false;
    boolean reversed2 = false;

    public Climb() {
    }

    public void move(double speed) {
        if (Math.abs(speed) < OperatorConstants.DEADBAND) {
            lifted1 = true;
            lifted2 = true;
            if (!reversed1) {
                leadForward *= -1;
                reversed1 = true;
            }
            if (!reversed2) {
                followForward *= -1;
                reversed2 = true;
            }
        } else {
            reversed1 = false;
            reversed2 = false;
        }
        speed = speed * 0.5;
        if (lifted2 == true || (FollowClimbLimit.get())) {
            m_follow.set(speed * followForward);
        } else {
            m_follow.set(0);
        }

        if (lifted1 == true || (LeadClimbLimit.get())) {
            m_lead.set(speed * leadForward);
        } else {
            m_lead.set(0);
        }
    }

    @Override
    public void periodic() {
        if (!LeadClimbLimit.get()) {
            if (!setClimb1) {
                lifted1 = false;
                setClimb1 = true;
            }

        } else {
            setClimb1 = false;
        }

        if (!FollowClimbLimit.get()) {
            if (!setClimb2) {
                lifted2 = false;
                setClimb2 = true;
            }
        } else {
            setClimb2 = false;
        }

        SmartDashboard.putBoolean("Lead Climb Limit Switch", LeadClimbLimit.get());
        SmartDashboard.putBoolean("Follow Climb Limit Switch", FollowClimbLimit.get());
    }

}
