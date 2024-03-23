package frc.robot.commands.swervedrive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodPositioner extends Command {
    public static double setpoint;
    public PIDController hoodPID = new PIDController(0.015, 0, 0);
    public CANSparkMax m_Leader = new CANSparkMax(HoodConstants.m_Lead, MotorType.kBrushless);
    public CANSparkMax m_Follower = new CANSparkMax(HoodConstants.m_Follow, MotorType.kBrushless);
    Hood hood;
    XboxController mechController;
    DigitalInput lowerLimitSwitch = new DigitalInput(23);
    DigitalInput upperLimitSwitch = new DigitalInput(10);

    public HoodPositioner(Hood self) {
        setpoint = 65;
        hood = self;
        mechController = RobotContainer.mechXbox;
        m_Follower.follow(m_Leader, true);
        addRequirements(self);
    }

    @Override
    public void execute() {
        // m_Leader.set(RobotContainer.mechXbox.getRightY()*0.2);
        if(hood.getRotation() != 0){
        try {
            if (Math.abs(mechController.getRightY()) > OperatorConstants.DEADBAND) {
                if ((mechController.getRightY() < 0 && hood.getRotation() > HoodConstants.lowerLimit)
                        || (mechController.getRightY() > 0 && hood.getRotation() < HoodConstants.upperLimit)) {
                    if ((mechController.getRightY() < 0 && lowerLimitSwitch.get())
                            || (mechController.getRightY() > 0 && upperLimitSwitch.get())) {
                        m_Leader.set(mechController.getRightY() * 0.3);
                    }
                    else{
                        m_Leader.set(0);
                    }
                } else {
                    m_Leader.set(0);
                }
                setpoint = hood.getRotation();
            } else {

                // clamp between min and max value
                setpoint = Math.max(HoodConstants.lowerLimit, Math.min(setpoint, HoodConstants.upperLimit));
                if ((hoodPID.calculate(hood.getRotation(), setpoint) < 0 && lowerLimitSwitch.get())
                        || (hoodPID.calculate(hood.getRotation(), setpoint) > 0 && upperLimitSwitch.get())) {
                    m_Leader.set(hoodPID.calculate(hood.getRotation(), setpoint));
                }
                else{
                    m_Leader.set(0);
                }
            }
        } catch (Exception e) {
            setpoint = Math.max(HoodConstants.lowerLimit, Math.min(setpoint, HoodConstants.upperLimit));
            m_Leader.set(hoodPID.calculate(hood.getRotation(), setpoint));

        }
    }
        SmartDashboard.putBoolean("Upper Limit Switch", upperLimitSwitch.get());
        SmartDashboard.putNumber("Hood Setpoint", setpoint);
        SmartDashboard.putBoolean("Lower Limit Switch", lowerLimitSwitch.get());

    }

}
