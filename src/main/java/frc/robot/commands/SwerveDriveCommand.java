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

    if(DriverStation.isAutonomous())
    {
      return;
    }

    // Scaled to 1/2 speed
    double Xj = XjSupplier.getAsDouble();
    double Yj = YjSupplier.getAsDouble();
    double Zj = ZjSupplier.getAsDouble();

    if(elevator.elevatorActive())
    {
      Xj = 0.5 * Xj;
      Yj = 0.5 * Yj;
      Zj = 0.5 * Zj;
    }

    // Min and max steering motor percent output
    double MinSteer = -1.0;
    double MaxSteer = 1.0;

    // Temporary variables for setpoint and control output
    double sp;
    double co;

    // Chassis dimensions
    double L = 21.5;
    double W = 24.5;
    double R = Math.sqrt(L*L + W*W);

    // Convert joystick values to strafe, forward, and rotate
    double deadband = 0.075;
    double STR = Xj;
    if (STR > -deadband && STR < deadband) STR = 0;
    else
    {
      if (STR > 0)
      {
        STR = (STR - deadband) / (1 - deadband);
      }
      else
      {
        STR = (STR + deadband) / (1 - deadband);
      }
    }

    double FWD = -Yj;
    if (FWD > -deadband && FWD < deadband) FWD = 0;
    else
    {
      if (FWD > 0)
      {
        FWD = (FWD - deadband) / (1 - deadband);
      }
      else
      {
        FWD = (FWD + deadband) / (1 - deadband);
      }
    }

    double RCW = Zj;
    if (RCW > -deadband && RCW < deadband) RCW = 0;
    else
    {
      if (RCW > 0)
      {
        RCW = (RCW - deadband) / (1 - deadband);
      }
      else
      {
        RCW = (RCW + deadband) / (1 - deadband);
      }
    }

    // Limit rotate speed
    RCW /= 2;

    /*if(elevator.elevatorActive())
    {
      RCW *=3;
    }*/

    // adjust for field oriented drive
    //double gyro_rad = (drivetrain.gyroYaw + m_angleChooser.getSelected()) / 180 * Math.PI;
    double gyro_rad = (drivetrain.gyroYaw + 180) / 180 * Math.PI;
    //double gyro_rad = drivetrain.gyroYaw / 180 * Math.PI;
    double tFWD = FWD * Math.cos(gyro_rad) + STR * Math.sin(gyro_rad);
    STR = -FWD * Math.sin(gyro_rad) + STR * Math.cos(gyro_rad);
    FWD = tFWD;

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

    // Otherwise stop motors
    else
    {
      for(int lp=0; lp<4; lp++)
      {
        drivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
        drivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
      }
    }

    /*
    System.out.printf("%10.2f %10.2f %10.2f %10.2f %10.2f %10.2f\n",
      Xj, Yj,
      rotation[0] * speed[0],
      rotation[1] * speed[1],
      rotation[2] * speed[2],
      rotation[3] * speed[3]
    );
    */

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
