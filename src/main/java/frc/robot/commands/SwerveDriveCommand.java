// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase 
{

  private final DoubleSupplier XjSupplier;
  private final DoubleSupplier YjSupplier;
  private final DoubleSupplier ZjSupplier;
  private final SwerveDriveSubsystem drivetrain;
  private final ElevatorSubsystem elevator;

  public SwerveDriveCommand(DoubleSupplier XjSupplier, DoubleSupplier YjSupplier, DoubleSupplier ZjSupplier, SwerveDriveSubsystem drivetrain, ElevatorSubsystem elevator) 
  {
    this.XjSupplier = XjSupplier;
    this.YjSupplier = YjSupplier;
    this.ZjSupplier = ZjSupplier;
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double deadband = 0.1;

    if(DriverStation.isAutonomous())
    {
      return;
    }

    double Xj = XjSupplier.getAsDouble();
    double Yj = YjSupplier.getAsDouble();
    double Zj = ZjSupplier.getAsDouble();

    if(elevator.elevatorActive())
    {
      Xj = 0.375 * Xj;
      Yj = 0.375 * Yj;
      Zj = 0.375 * Zj;
    }

    if (Xj > -deadband && Xj < deadband) Xj = 0;
    else
    {
      if (Xj > 0)
      {
        Xj = (Xj - deadband) / (1 - deadband);
      }
      else
      {
        Xj = (Xj + deadband) / (1 - deadband);
      }
    }

    if (Yj > -deadband && Yj < deadband) Yj = 0;
    else
    {
      if (Yj > 0)
      {
        Yj = (Yj - deadband) / (1 - deadband);
      }
      else
      {
        Yj = (Yj + deadband) / (1 - deadband);
      }
    }

    if (Zj > -deadband && Zj < deadband) Zj = 0;
    else
    {
      if (Zj > 0)
      {
        Zj = (Zj - deadband) / (1 - deadband);
      }
      else
      {
        Zj = (Zj + deadband) / (1 - deadband);
      }
    }

    drivetrain.driveSwerve(Xj, Yj, Zj, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if(DriverStation.isAutonomous())
    {
      return;
    }
    drivetrain.driveSwerve(0, 0, 0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
