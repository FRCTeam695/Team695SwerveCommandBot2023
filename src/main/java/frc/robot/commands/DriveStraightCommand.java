// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveStraightCommand extends CommandBase 
{
  private final SwerveDriveSubsystem drivetrain;
  private final double driveSpeed;
  private final double ticksToDrive;

  double initialTicks;
  double initialRobotYaw;

  /** Creates a new DriveStraightCommand. */
  public DriveStraightCommand(SwerveDriveSubsystem drivetrain, double ticksToDrive, double driveSpeed) 
  {
    this.drivetrain = drivetrain;
    this.driveSpeed = driveSpeed;
    this.ticksToDrive = ticksToDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    initialRobotYaw = drivetrain.gyroYaw;
    initialTicks = drivetrain.drive[0].getSelectedSensorPosition();
    System.out.println("DRIVE STRAIGHT INITIALIZED");
    //drivetrain.drive[0].setSelectedSensorPosition(0, 0,100);
  }

  @Override
  public void execute() 
  {
    drivetrain.driveStraight(driveSpeed, initialRobotYaw, initialTicks);
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
  public boolean isFinished() 
  {
    double deltaTicks = Math.abs(initialTicks - drivetrain.drive[0].getSelectedSensorPosition(0));
    if (deltaTicks >= ticksToDrive)
    {
      System.out.println("DRIVE STRAIGHT HAS ENDED");
      return true;
    }
    return false;
  }  
}
