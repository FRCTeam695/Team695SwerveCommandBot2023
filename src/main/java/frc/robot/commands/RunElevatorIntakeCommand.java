// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorIntakeSubsystem;

public class RunElevatorIntakeCommand extends CommandBase 
{
  private final ElevatorIntakeSubsystem m_ElevatorIntakeSubsystem;
  double direction;

  public RunElevatorIntakeCommand(ElevatorIntakeSubsystem elevatorIntakeSubsystem, double direction) 
  {
    this.m_ElevatorIntakeSubsystem = elevatorIntakeSubsystem;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ElevatorIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_ElevatorIntakeSubsystem.runInit(direction);
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
    // only explicitly stop if running out (running in is a toggle on/off)
    if (direction == 1)
    {
      m_ElevatorIntakeSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.isAutonomous())
    {
      if(m_ElevatorIntakeSubsystem.getStallHold())
      {
        return true;
      }
      return false;
    }
    // running in is a toggle, so set to immediately exit
    if (direction == -1)
    {
      return(true);
    }
    return false;
  }
}
