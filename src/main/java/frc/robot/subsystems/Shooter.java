package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.swervedrive.HoodPositioner;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    CANSparkFlex m_leader = new CANSparkFlex(ShooterConstants.m_LeaderID, MotorType.kBrushless);
    CANSparkFlex m_Follower = new CANSparkFlex(ShooterConstants.m_FollowerID, MotorType.kBrushless);

    public CANSparkMax index = new CANSparkMax(ShooterConstants.m_IndexID, MotorType.kBrushless);

    public SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    private PolynomialSplineFunction m_shooterAngleCurve;
    private PolynomialSplineFunction m_shooterFlywheelCurve;
    public double setpoint = 0;
    private final Timer m_timer = new Timer();

    // Ultrasonic rangeFinder = new Ultrasonic(3, 4);

    // Key: Distance
    // Value: Angle

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

        var shooterMap = ShooterConstants.SHOOTER_MAP;
        double[] distances = new double[shooterMap.size()];
        double[] flywheelSpeeds = new double[shooterMap.size()];
        double[] angles = new double[shooterMap.size()];
    
        for (int i = 0; i < shooterMap.size(); i++) {
          distances[i] = shooterMap.get(i).getKey();
          flywheelSpeeds[i] = shooterMap.get(i).getValue().speed;
          angles[i] = shooterMap.get(i).getValue().angle;
        }

        m_shooterFlywheelCurve = SPLINE_INTERPOLATOR.interpolate(distances, flywheelSpeeds);
        m_shooterAngleCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
    
    }

    public void RunShooter(double speed, boolean auto) {
        setpoint =  speed * maxRPM;
        
        if (RobotContainer.mechXbox.getAButton() || RobotContainer.mechXbox.getBButton()) {
            index.set(-1);
        } 
        else if (RobotContainer.mechXbox.getXButton()) {
            index.set(0.5);
        } else {
            index.set(0);
        }
        
        if(RobotContainer.mechXbox.getRightBumper() || auto){
            setpoint = getAutomaticState().speed;
            HoodPositioner.setpoint = getAutomaticState().angle;
        }
        SmartDashboard.putNumber("Shooter SetPoint", setpoint);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }

    public double getVelocity(){
        return m_encoder.getVelocity();
    }

    public void quickReverse() {
        m_timer.reset();
        m_timer.start();
        while (m_timer.get()<0.2) {
            index.set(0.5);
        }
    }

    public Command AutoShoot() {
        return new RunCommand(() -> {
            quickReverse();
            RunShooter(0, true);
        }
            , this);
    }

    public State getAutomaticState() {
        var targetDistance = getTargetDistance();
         var flywheelSpeed = m_shooterFlywheelCurve.value(targetDistance);
         var angle = m_shooterAngleCurve.value(targetDistance);
    
         return new State(flywheelSpeed ,angle);
      }

    private double getTargetDistance() {
        return Vision.distance;
    }

    @Override
    public void periodic() {
        m_pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public static class State {
        public final double speed;
        public final double angle;

        public State(double speed, double angle) {
            this.speed = speed;
            this.angle = angle;
        }
    }

    public Command Fire() {
        return new RunCommand(() -> index.set(-1), this);
    }
    public Command StopShooter(){
        return new RunCommand(()->{
            RunShooter(0, false);
            index.set(0);
        }, this);
    }
}
