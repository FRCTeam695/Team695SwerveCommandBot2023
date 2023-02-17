// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIntakeSubsystem extends SubsystemBase 
{
  private CANSparkMax m_NEO550Motor = new CANSparkMax(9, MotorType.kBrushless);

  long currentIntakeMode;

  public ElevatorIntakeSubsystem() 
  {
    m_NEO550Motor.setIdleMode(IdleMode.kBrake);
  }

  public void setNEOMotorSpeed(double speed)
  {
    currentIntakeMode = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentIntakeMode").getInteger(-1);
    if (currentIntakeMode == 1)
    {
      m_NEO550Motor.setSmartCurrentLimit(25);
      m_NEO550Motor.set(speed);
    }
    else if(currentIntakeMode == 2)
    {
      m_NEO550Motor.setSmartCurrentLimit(25);
      m_NEO550Motor.set(-0.7*speed);
    }
  }

  @Override
  public void periodic() 
  {
  }
}
