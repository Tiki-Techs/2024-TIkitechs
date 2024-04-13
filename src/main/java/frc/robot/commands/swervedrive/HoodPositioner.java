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
    DigitalInput lowerLimitSwitch = new DigitalInput(2);
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
        // first check if the encoder is equal to 0. It should only be zero if it is
        // unplugged.
        // and if it unplugged, we should stop running the hood.
        if (hood.rotEncoder.getAbsolutePosition() != 0) {
            try {

                // Checks if the right stick on the mech controller is being moved by lani
                if (Math.abs(mechController.getRightY()) > OperatorConstants.DEADBAND) {

                    // if it is, we make sure that we arent trying to move the hood into itself.
                    // ie if getRightY() is negative, we need to make sure that the hood is above
                    // its lower limit otherwise it will run into itself.
                    if ((mechController.getRightY() < 0 && hood.getRotation() > HoodConstants.lowerLimit)
                            || (mechController.getRightY() > 0 && hood.getRotation() < HoodConstants.upperLimit)) {

                        // check that the limit switches arent pressed as a backup
                        if ((mechController.getRightY() < 0 && lowerLimitSwitch.get())
                                || (mechController.getRightY() > 0 && upperLimitSwitch.get())) {
                            // manually control the hood
                            m_Leader.set(mechController.getRightY() * 0.3);
                        } else {
                            // set the motor to zero when it is not being moved.
                            m_Leader.set(0);
                        }
                    } else {
                        // set the motor to zero when it is not being moved.
                        m_Leader.set(0);
                    }
                    // set the new setpoint of the hood to where the hood currently is, that way the
                    // pid isnt fighting lani if she is controlling it manually.
                    setpoint = hood.getRotation();
                }

                else {

                    // clamp setpoint between min and max value so it doesnt try to go to a position
                    // it cant reach and run into itself
                    setpoint = Math.max(HoodConstants.lowerLimit, Math.min(setpoint, HoodConstants.upperLimit));

                    // makes sure that its not trying to go in the direction of a pressed limit
                    // switch
                    if ((hoodPID.calculate(hood.getRotation(), setpoint) < 0 && lowerLimitSwitch.get())
                            || (hoodPID.calculate(hood.getRotation(), setpoint) > 0 && upperLimitSwitch.get())) {
                        // sets the motor to the value of the pid.
                        m_Leader.set(hoodPID.calculate(hood.getRotation(), setpoint));
                    } else {
                        // sets the motor to zero if it is trying to go towards a pressed limit switch.
                        m_Leader.set(0);
                    }
                }
            } catch (Exception e) {
                m_Leader.set(0);

            }
        } else {
            // sets the hood to stop running if the encoder is unplugged.
            m_Leader.set(0);
        }
        SmartDashboard.putBoolean("Upper Limit Switch", upperLimitSwitch.get());
        SmartDashboard.putNumber("Hood Setpoint", setpoint);
        SmartDashboard.putBoolean("Lower Limit Switch", lowerLimitSwitch.get());

    }

}
