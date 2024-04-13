package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.swervedrive.HoodPositioner;

public class Intake extends SubsystemBase {
    public static CANSparkMax mot = new CANSparkMax(IntakeConstants.m_Intake, MotorType.kBrushless);
    private final Timer m_timer = new Timer();

    //this method is the default command of this subsystem, so it is always repeating and running
    public void run() {
        
            //if the A button is pressed run this
        if (RobotContainer.mechXbox.getAButton()) {
            //Dont run the intake until the hood is at an angle of more than 60 degrees
            //this is so we dont intake a note below the index and it gets stuck.
            if (RobotContainer.s_Hood.getRotation() > 60) {
                mot.set(1);
            }
            //sets the hood to move to 66 degrees. 
            HoodPositioner.setpoint = 66;
        } else if (RobotContainer.mechXbox.getYButton()) {
            //the y button is opposite to the A button, so we want it to reverse the intake so we can spit something out if it gets stuck. 
            mot.set(-1);
        } else {
            //if neither are pressed stop running the intake
            mot.set(0);
        }
    }


    // sets the intake motor speed to 1 and the index to -1. This is how we pick up notes. 
    // When using .set() the motor will stay at that speed until it is told to change
    public Command StartIntake() {
        return new InstantCommand(() -> {
            mot.set(1);
            RobotContainer.s_Shooter.index.set(-1);
        }, this);
    }


    public Command StopIntake() {
        return new InstantCommand(() -> {
            m_timer.reset();
            m_timer.start();

            //runs for 0.5 seconds
            while (m_timer.get() < 0.5) {
                //keeps the intake running for half a second so that we make sure the note gets picked up all the way.
                mot.set(1);
                RobotContainer.s_Shooter.index.set(-1);
            }
            // stops the intake motor
            mot.set(0);

            //runs for 0.2 seconds
            while (m_timer.get() < 0.7) {
                // runs the index motor backwards for 0.2 seconds so that the note isnt pressed up against the shooter wheels
                // and they can spin freely. 
                RobotContainer.s_Shooter.index.set(0.5);
            }
            //stops the index. 
            RobotContainer.s_Shooter.index.set(0);

        }, this);
    }
}
