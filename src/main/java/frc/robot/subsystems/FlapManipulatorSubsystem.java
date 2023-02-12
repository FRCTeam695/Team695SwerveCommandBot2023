// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlapManipulatorSubsystem extends SubsystemBase 
{
  private CANSparkMax m_flap = new CANSparkMax(51, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_flap.getEncoder();

  public FlapManipulatorSubsystem() 
  {
    m_flap.setIdleMode(IdleMode.kBrake);
    m_flap.setSmartCurrentLimit(25);
    clearPosition();
  }

  public void setSpeed(double speed)
  {
    m_flap.set(speed);
  }

  public void clearPosition()
  {
    //m_encoder1 = sparkMax1.getEncoder();
    m_encoder.setPosition(0);

  }

  public double getPosition()
  {
    return(m_encoder.getPosition());
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("FlapPosition", getPosition());
  }
}
