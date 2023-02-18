// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
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
  private final SendableChooser<Double> m_angleChooser;
  private final double ticksToStrafe;
  private final double strafeSpeed;
  private final double adjustmentTicks;

  double initialRobotYaw;
  
  public StrafeToTargetCommand(SwerveDriveSubsystem drivetrain, VisionSubsystem visionSubsystem, SendableChooser<Double> angleChooser, double ticksToStrafe, double strafeSpeed, double adjustmentTicks) 
  {
    this.drivetrain = drivetrain;
    this.visionSubsystem = visionSubsystem;
    this.m_angleChooser = angleChooser;
    this.ticksToStrafe = ticksToStrafe;
    this.strafeSpeed = strafeSpeed;
    this.adjustmentTicks = adjustmentTicks;
    
    addRequirements(drivetrain, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    initialRobotYaw = drivetrain.gyroYaw;
    drivetrain.drive[0].setSelectedSensorPosition(0);
  }

  public void driveStrafe(double adjXj)
  {
    double gyroError = initialRobotYaw - drivetrain.gyroYaw;

    double adjZj = gyroError * (0.05);

    // Min and max steering motor percent output
    double MinSteer = -1.0;
    double MaxSteer = 1.0;

    // Temporary variables for setpoint and control output
    double sp;
    double co;

    // Chassis dimensions
    double L = 25;
    double W = 25;
    double R = Math.sqrt(L*L + W*W);

    // Convert joystick values to strafe, forward, and rotate
    double deadband = 0.025;  // Originally 0.075
    double STR = adjXj; 
    if (STR > -deadband && STR < deadband) STR = 0;

    double rotationDeadband = 0.0125;
    double RCW = adjZj;
    if (RCW > -rotationDeadband && RCW < rotationDeadband) RCW = 0;

    RCW /= 3;

    double FWD = 0;
    
    // adjust for field oriented drive
    //double gyro_rad = (drivetrain.gyroYaw + m_angleChooser.getSelected()) / 180 * Math.PI;
    double gyro_rad = drivetrain.gyroYaw / 180 * Math.PI;
    double tFWD = STR * Math.sin(gyro_rad);
    STR = STR * Math.cos(gyro_rad);
    FWD = tFWD;

    SmartDashboard.putNumber("aSTR", STR);
    SmartDashboard.putNumber("aFWD", FWD);
    
    // Compute temporary work variables
    double A = drivetrain.nearzero(STR + RCW * (L/R));
    double B = drivetrain.nearzero(STR - RCW * (L/R));
    double C = drivetrain.nearzero(FWD + RCW * (W/R));
    double D = drivetrain.nearzero(FWD - RCW * (W/R));

    // Compute angles (range is -180 to 180 degrees)
    double[] angle =
    {
      drivetrain.nearzero(Math.atan2(A,D) * 180 / Math.PI),
      drivetrain.nearzero(Math.atan2(A,C) * 180 / Math.PI),
      drivetrain.nearzero(Math.atan2(B,C) * 180 / Math.PI),
      drivetrain.nearzero(Math.atan2(B,D) * 180 / Math.PI)
    };

    // Compute speeds
    double[] speed =
    {
      Math.sqrt(A*A + D*D),
      Math.sqrt(A*A + C*C),
      Math.sqrt(B*B + C*C),
      Math.sqrt(B*B + D*D)
    };

    // Set default drive rotations
    double[] rotation =
    {
      drivetrain.defaultrotation[0],
      drivetrain.defaultrotation[1],
      drivetrain.defaultrotation[2],
      drivetrain.defaultrotation[3]
    };

    // Normalize speeds for motor controllers (range is 0 to 1)
    double max = 0;
    for(int lp=0; lp<4; lp++) if (speed[lp] > max) max = speed[lp];
    if (max > 1) for(int lp=0; lp<4; lp++) speed[lp] /= max;
    
    // Only run motors if nonzero speed(s)
    if (max != 0)
    {

      // Loop to compute and apply steering angle and drive power to each motor
      for(int lp=0; lp<4; lp++)
      {

        // Get target steering angle (setpoint) for this corner
        // (Invert because falcons are upside down)
        sp = angle[lp];

        // get current steering angle (process variable) for this corner
        double ec = drivetrain.steer[lp].getSelectedSensorPosition(0);
        double pv = (ec % Constants.talon_mk4i_360_count) / Constants.talon_mk4i_360_count * 360;
        if (pv <= -180)
        {
          pv += 360;
        }
        else if (pv >= 180)
        {
          pv -= 360;
        }

        // Compute steering angle change
        double delta = Math.abs(sp - pv);

        // Adjust steering angle for optimized turn
        if (delta > 90 && delta < 270)
        {
          rotation[lp] = -drivetrain.defaultrotation[lp];
          sp = (sp >= 0) ? sp - 180 : sp + 180;
        }

        // Adjust angle orientation for PID depending on direction of setpoint
        if (sp < -135 || sp > 135)
        {
          if (sp < 0)
          {
            sp += 360;
          }
          if (pv < 0)
          {
            pv += 360;
          }
        }

        co = MathUtil.clamp(drivetrain.talonpid[lp].calculate(pv, sp), MinSteer, MaxSteer);

        // Send steering output to falcon
        drivetrain.steer[lp].set(ControlMode.PercentOutput, co);

        // Send drive output to falcon
        drivetrain.drive[lp].set(ControlMode.PercentOutput, rotation[lp] * speed[lp]);

        // Convenient place to do cancoder home control loop cleanup
        drivetrain.cancoderpid[lp].reset();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      driveStrafe(strafeSpeed);
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
    SmartDashboard.putNumber("xEC", ec);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    double adjustedTicksToStrafe = ticksToStrafe + adjustmentTicks;
    if (Math.abs(drivetrain.drive[0].getSelectedSensorPosition(0)) >= adjustedTicksToStrafe)    // Previously 87000
    {
      System.out.println("Strafe isFinished has returned true");
      return true;
    }
    return false;
  }
}