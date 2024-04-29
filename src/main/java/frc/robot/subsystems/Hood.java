package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.commands.swervedrive.HoodPositioner;

public class Hood extends SubsystemBase {

    public DutyCycleEncoder rotEncoder = new DutyCycleEncoder(HoodConstants.EncoderID);
    public PIDController hoodPID = new PIDController(0.002, 0, 0);

    public Hood() {
        SmartDashboard.putNumber("Hood Setpoint", 65);
    }

    public double getRotation() {
        // is what is used by other classes to get the rotation of the hood.
        // converts the absolute encoder (0 to 1) to degrees (0 to 360) and subtracts
        // the encoder offset.
        return Rotation2d.fromRotations(rotEncoder.getAbsolutePosition()).getDegrees() - HoodConstants.encoderOffset;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood PID", hoodPID.calculate(getRotation(), HoodPositioner.setpoint));
        SmartDashboard.putNumber("Hood Setpoint", HoodPositioner.setpoint);
        SmartDashboard.putNumber("Hood Angle", getRotation());

    }

}
