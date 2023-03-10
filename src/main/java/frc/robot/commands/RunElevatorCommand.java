// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RunElevatorCommand extends CommandBase 
{
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final ElevatorIntakeSubsystem m_ElevatorIntakeSubsystem;

  public RunElevatorCommand(ElevatorSubsystem elevatorSubsystem, ElevatorIntakeSubsystem elevatorIntakeSubsystem) 
  {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_ElevatorIntakeSubsystem = elevatorIntakeSubsystem;
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    long currentLevel = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentLevel").getInteger(-1);
    if(m_ElevatorIntakeSubsystem.getStallHold())
    {
      m_ElevatorSubsystem.runToLevel((int)currentLevel);
    }
    else
    {
      m_ElevatorSubsystem.runToLevel(3);
    }
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
