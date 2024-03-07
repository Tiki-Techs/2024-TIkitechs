package frc.robot.subsystems;

//for dictionary

import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    CANSparkFlex m_leader = new CANSparkFlex(ShooterConstants.m_LeaderID, MotorType.kBrushless);
    CANSparkFlex m_Follower = new CANSparkFlex(ShooterConstants.m_FollowerID, MotorType.kBrushless);

    CANSparkMax index = new CANSparkMax(ShooterConstants.m_IndexID, MotorType.kBrushless);

    private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    // Ultrasonic rangeFinder = new Ultrasonic(3, 4);

    // Key: Distance
    // Value: Angle
    Dictionary<Double, Double> dictionary = new Hashtable<>();

    public Shooter() {
    m_Follower.follow(m_leader, true);
     m_pidController = m_leader.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_leader.getEncoder();

    // PID coefficients
    kP = 0.0018; 
    kI = 0;
    kD = 0.07591; 
    kIz = 0; 
    kFF = 0.0001; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
        // dictionary.put(null, null);
    }

    public void RunShooter(double speed) {
        var setpoint = maxRPM*speed;
        m_pidController.setReference(setpoint, ControlType.kVelocity);
        SmartDashboard.putNumber("SetPoint", setpoint);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
        if (RobotContainer.mechXbox.getAButton()) {
            index.set(-1);
        } else if (RobotContainer.mechXbox.getYButton()) {
            index.set(-0.5);
        } else {
            index.set(0);
        }
    }

    public void RunIndex() {
    }

    public Command StartShooter() {
        return new RunCommand(() -> RunShooter(1), this);
    }

    public Command StopShooter() {
        return new RunCommand(() -> RunShooter(0), this);
    }

    @Override
    public void periodic() {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
    
    }

}
