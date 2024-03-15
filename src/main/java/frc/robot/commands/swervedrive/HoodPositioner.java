package frc.robot.commands.swervedrive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodPositioner extends Command {
    public static double setpoint;
    public PIDController hoodPID = new PIDController(0.015, 0, 0);
    public CANSparkMax m_Leader = new CANSparkMax(HoodConstants.m_Lead, MotorType.kBrushless);
    Hood hood;
    XboxController mechController;

    public HoodPositioner(Hood self) {
        setpoint = 65;
        hood = self;
        mechController = RobotContainer.mechXbox;
        addRequirements(self);
    }

    @Override
    public void execute() {
        // m_Leader.set(RobotContainer.mechXbox.getRightY()*0.2);
        try{
        if (Math.abs(mechController.getRightY()) > OperatorConstants.DEADBAND) {
            if ((mechController.getRightY() < 0 && hood.getRotation() > HoodConstants.lowerLimit)
                    || (mechController.getRightY() > 0 && hood.getRotation() < HoodConstants.upperLimit)) {
                m_Leader.set(mechController.getRightY() * 0.3);
            } else {
                m_Leader.set(0);
            }
            setpoint = hood.getRotation();
        } else {

            // clamp between min and max value
            setpoint = Math.max(HoodConstants.lowerLimit, Math.min(setpoint, HoodConstants.upperLimit));
            m_Leader.set(hoodPID.calculate(hood.getRotation(), setpoint));
        }
        }catch(Exception e){
            setpoint = Math.max(HoodConstants.lowerLimit, Math.min(setpoint, HoodConstants.upperLimit));
            m_Leader.set(hoodPID.calculate(hood.getRotation(), setpoint));

        } 
    }

    
}
