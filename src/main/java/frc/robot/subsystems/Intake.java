package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    public static CANSparkMax mot = new CANSparkMax(IntakeConstants.m_Intake, MotorType.kBrushless);

    public void run(double speed)
    {
        mot.set(speed);
    }

    public Command StartIntake(){
        return new RunCommand(() -> run(1), this);
    }

    public Command StopIntake(){
        return new RunCommand(() -> run(0), this);
    }
}
