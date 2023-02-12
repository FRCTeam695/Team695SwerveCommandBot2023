// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIntakeSubsystem extends SubsystemBase 
{
  private CANSparkMax m_NEO550Motor = new CANSparkMax(9, MotorType.kBrushless);

  public ElevatorIntakeSubsystem() 
  {
    m_NEO550Motor.setIdleMode(IdleMode.kBrake);
    m_NEO550Motor.setSmartCurrentLimit(25);
  }

  public void setNEOMotorSpeed(double speed)
  {
    m_NEO550Motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
