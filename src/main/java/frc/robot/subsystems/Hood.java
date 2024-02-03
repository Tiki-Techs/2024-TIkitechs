// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Hood extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //  public CANSparkMax hoodMotor = new CANSparkMax(2, MotorType.kBrushless);
    //public CANSparkMax hoodFollower = new CANSparkMax(3, MotorType.kBrushless);

    //public Encoder h_Encoder = new Encoder(Constants.HoodConstants.encoderA, Constants.HoodConstants.encoderB);

    public Hood() {
      //hoodFollower.follow(hoodMotor);
    }

  public void RunMotor(double speed){
    //hoodFollower.set(speed *-0.75);
    //hoodMotor.set(speed*0.75);
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Hood Encoder", h_Encoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
