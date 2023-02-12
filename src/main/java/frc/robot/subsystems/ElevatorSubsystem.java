// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase 
{
  private final WPI_TalonFX m_ElevatorFalcon = new WPI_TalonFX(2);
  
  public double elevatorPosition;
  public double elevatorSpeed;

  private int cnt = 0;

  public ElevatorSubsystem() 
  {
    m_ElevatorFalcon.setNeutralMode(NeutralMode.Brake);
    SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
    falconlimit.enable = true;
    falconlimit.currentLimit = 10;
    falconlimit.triggerThresholdCurrent = 10;
    falconlimit.triggerThresholdTime = 0;
    m_ElevatorFalcon.setSelectedSensorPosition(0);
  }

  public void setSpeed(double percentVBus)
  {
    double RPM = m_ElevatorFalcon.getSelectedSensorVelocity() * 600 / 2048;
    SmartDashboard.putNumber("RPM", RPM);
    m_ElevatorFalcon.set(percentVBus);
  }

  public double getPosition()
  {
    return(m_ElevatorFalcon.getSelectedSensorPosition());
  }

  @Override
  public void periodic() 
  {
  }
}
