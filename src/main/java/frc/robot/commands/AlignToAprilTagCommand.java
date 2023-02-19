// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTagCommand extends CommandBase 
{
  private final SwerveDriveSubsystem drivetrain;
  private final VisionSubsystem m_VisionSubsystem;

  double initialTicks;
  double initialRobotYaw;

  public AlignToAprilTagCommand(SwerveDriveSubsystem drivetrain, VisionSubsystem visionSubsystem) 
  {
    this.drivetrain = drivetrain;
    this.m_VisionSubsystem = visionSubsystem;

    addRequirements(drivetrain, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    initialRobotYaw = drivetrain.gyroYaw;
    initialTicks = drivetrain.drive[0].getSelectedSensorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_VisionSubsystem.hasTarget() == true)
    {
      if(m_VisionSubsystem.getPitch() < 7)
      {
        drivetrain.driveStraight(-0.15, initialRobotYaw, initialTicks);
      }
      else if (m_VisionSubsystem.getYaw() < 13)
      {
        drivetrain.driveStrafe(0.15, initialRobotYaw, initialTicks);
      }
      else
      {
        for(int lp=0; lp<4; lp++)
        {
          //drivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
          drivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    for(int lp=0; lp<4; lp++)
    {
      drivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
      drivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
