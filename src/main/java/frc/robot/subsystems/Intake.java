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

    public void run() {
        if (RobotContainer.mechXbox.getAButton()) {
            if (RobotContainer.s_Hood.getRotation() > 60) {
                mot.set(1);
            }
            HoodPositioner.setpoint = 66;
        } else if (RobotContainer.mechXbox.getYButton()) {
            mot.set(-0.5);
        } else {
            mot.set(0);
        }
    }

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
            while (m_timer.get() < 0.5) {
                mot.set(1);
                RobotContainer.s_Shooter.index.set(-1);
            }
            mot.set(0);

            while (m_timer.get() < 0.7) {
                RobotContainer.s_Shooter.index.set(0.5);
            }
            RobotContainer.s_Shooter.index.set(0);

        }, this);
    }
}
