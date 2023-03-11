// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RunElevatorCommand extends CommandBase 
{
  private final ElevatorSubsystem m_ElevatorSubsystem;

  public RunElevatorCommand(ElevatorSubsystem elevatorSubsystem) 
  {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_ElevatorSubsystem.runToLevel();
    /*
    if(m_ElevatorIntakeSubsystem.getStallHold())
    {
      m_ElevatorSubsystem.runToLevel((int)currentLevel);
    }
    else
    {
      m_ElevatorSubsystem.runToLevel(3);
    }
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
  
}
