// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class StrafeToTargetCommand extends CommandBase 
{
  private final SwerveDriveSubsystem drivetrain;
  private final VisionSubsystem visionSubsystem;
  //private final double ticksToStrafe;
  private final double strafeSpeed;
  private final double adjustmentTicks;

  double initialTicks;
  double initialRobotYaw;
  long scorePosition;
  double ticksToStrafe;
  
  public StrafeToTargetCommand(SwerveDriveSubsystem drivetrain, VisionSubsystem visionSubsystem, double strafeSpeed, double adjustmentTicks) 
  {
    this.drivetrain = drivetrain;
    this.visionSubsystem = visionSubsystem;
    //this.ticksToStrafe = ticksToStrafe;
    this.strafeSpeed = strafeSpeed;
    this.adjustmentTicks = adjustmentTicks;
    
    addRequirements(drivetrain, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    scorePosition = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentGrid").getInteger(-1);
    ticksToStrafe = (((double)scorePosition )* 30300) + 3000;       // Conversion factor previously 26400
    //SmartDashboard.putNumber("TravelTicks", ticksToStrafe);

    initialRobotYaw = drivetrain.gyroYaw;
    initialTicks = drivetrain.drive[0].getSelectedSensorPosition();
    //drivetrain.drive[0].setSelectedSensorPosition(0,0,100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      drivetrain.driveStrafe(strafeSpeed, initialRobotYaw, initialTicks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Strafe is ending");
      
    for(int lp=0; lp<4; lp++)
    {
      drivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
      drivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
    }
    double ec = drivetrain.drive[0].getSelectedSensorPosition(0);
    //SmartDashboard.putNumber("xEC", ec);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    double adjustedTicksToStrafe = ticksToStrafe + adjustmentTicks;
    double deltaTicks = Math.abs(initialTicks - drivetrain.drive[0].getSelectedSensorPosition(0));
    if (deltaTicks >= adjustedTicksToStrafe)    // Previously 87000
    {
      System.out.println("Strafe isFinished has returned true");
      return true;
    }
    return false;
  }
}