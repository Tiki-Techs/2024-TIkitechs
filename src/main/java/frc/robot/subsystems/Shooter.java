package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.swervedrive.HoodPositioner;

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
        setpoint = speed * maxRPM;

        if (RobotContainer.mechXbox.getAButton() || RobotContainer.mechXbox.getBButton()) {
            index.set(-1);
        } else if (RobotContainer.mechXbox.getXButton()) {
            index.set(0.5);
        } else {
            index.set(0);
        }

        if (RobotContainer.mechXbox.getRightBumper() || auto) {
            if (Vision.distance != 0) {
                setpoint = getAutomaticState().speed;
                HoodPositioner.setpoint = getAutomaticState().angle;
            } else {
                setpoint = 5700;
            }
        }
        if (RobotContainer.mechXbox.getPOV() == 0) {
            setpoint = 5700;
            HoodPositioner.setpoint = Constants.HoodConstants.LobAngle;
        }
        SmartDashboard.putNumber("Shooter SetPoint", setpoint);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void quickReverse() {
        m_timer.reset();
        m_timer.start();
        while (m_timer.get() < 0.2) {
            index.set(0.5);
        }
    }

    public State getAutomaticState() {
        var targetDistance = getTargetDistance();
        var flywheelSpeed = m_shooterFlywheelCurve.value(targetDistance);
        var angle = m_shooterAngleCurve.value(targetDistance);

        return new State(flywheelSpeed, angle);
    }

    private double getTargetDistance() {
        return Vision.distance;
    }

    @Override
    public void periodic() {
        if (setpoint != 0) {
            m_pidController.setReference(setpoint, ControlType.kVelocity);
        } else {
            m_leader.set(0);
        }
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
        return new InstantCommand(
                () -> {
                    Timer a_timer = new Timer();
                    a_timer.reset();
                    a_timer.start();
                    while (a_timer.get() < 0.5) {
                        m_pidController.setReference(5700, ControlType.kVelocity);
                        if (Vision.distance != 0) {
                            var set = Math.max(HoodConstants.lowerLimit,
                                    Math.min(getAutomaticState().angle, HoodConstants.upperLimit));
                            RobotContainer.positioner.m_Leader
                                    .set(RobotContainer.positioner.hoodPID
                                            .calculate(RobotContainer.s_Hood.getRotation(), set));
                        } else {
                            RobotContainer.positioner.m_Leader.set(0);
                        }
                        index.set(-1);
                    }
                    index.set(0);
                }, this);
    }

    public Command StopShooter() {
        return new InstantCommand(() -> {
            Timer a_timer = new Timer();
            a_timer.reset();
            a_timer.start();
            m_leader.set(0);
            while (a_timer.get() < 0.5) {
                m_pidController.setReference(0, ControlType.kVelocity);
                m_leader.set(0);
                if (RobotContainer.s_Hood.getRotation() < 67) {
                    RobotContainer.positioner.m_Leader
                            .set(RobotContainer.positioner.hoodPID.calculate(RobotContainer.s_Hood.getRotation(), 67));
                } else {
                    RobotContainer.positioner.m_Leader.set(0);
                }
            }
            RobotContainer.positioner.m_Leader.set(0);
        }, this);
    }

    public Command AimAuto() {
        return new InstantCommand(() -> {
            Timer a_timer = new Timer();
            a_timer.reset();
            a_timer.start();
            while (a_timer.get() < 1) {
                m_pidController.setReference(5700, ControlType.kVelocity);

                RobotContainer.drivebase.drive(new Translation2d(0,
                        0),
                        0,
                        true);

                if (Vision.distance != 0) {
                    var set = Math.max(HoodConstants.lowerLimit,
                            Math.min(getAutomaticState().angle, HoodConstants.upperLimit));
                    RobotContainer.positioner.m_Leader
                            .set(RobotContainer.positioner.hoodPID.calculate(RobotContainer.s_Hood.getRotation(), set));
                } else {
                    RobotContainer.positioner.m_Leader.set(0);
                }
            }
        }, this);
    }
}
