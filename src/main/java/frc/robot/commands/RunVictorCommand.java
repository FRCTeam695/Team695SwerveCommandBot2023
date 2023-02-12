// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RunVictorCommand extends CommandBase {
  /** Creates a new RunVictor. */
  private final VictorSPXSubsystem m_VictorSPXSubsystem;
  private double victorSpeed;


  public RunVictorCommand(VictorSPXSubsystem victorSPX, double speed) 
  {
    m_VictorSPXSubsystem = victorSPX;
    victorSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_VictorSPXSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_VictorSPXSubsystem.setVictorSpeed(victorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_VictorSPXSubsystem.setVictorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
