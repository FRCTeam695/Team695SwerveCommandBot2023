// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class VictorSPXSubsystem extends SubsystemBase 
{
  /** Creates a new VictorSPX. */
  private final WPI_VictorSPX m_Victor = new WPI_VictorSPX(7);    

  public VictorSPXSubsystem() {}

  public void setVictorSpeed(double speed) 
  {
    m_Victor.setNeutralMode(NeutralMode.Brake);
    m_Victor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
