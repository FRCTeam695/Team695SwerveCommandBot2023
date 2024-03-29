// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.*;

public class SwerveDriveSubsystem extends SubsystemBase 
{
  private final ElevatorSubsystem elevator;

  // Navx2 gyro
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Digital Input to determine robot
  public DigitalInput robotDigitalInput = new DigitalInput(9);

  public double gyroYaw;
  public double gyroPitch;
  public double gyroAngle;

  // Cancoders
  public static WPI_CANCoder cancoder[] =
  {
    new WPI_CANCoder(11, "drivetrain"),
    new WPI_CANCoder(21, "drivetrain"),
    new WPI_CANCoder(31, "drivetrain"),
    new WPI_CANCoder(41, "drivetrain")
  };

  // Cancoder mounting orientation offsets (degrees)
  
 // summer 2022 bot:  static double[] cancoderoffset = { 170, 228, 170, 204 };
  static double[] cancoderoffset = { 158, 222, 247, 29 };
  //static double[] cancoderoffset = new double[4];

  // Steering motors
  public static TalonFX steer[] = 
  {
    new TalonFX(12, "drivetrain"),
    new TalonFX(22, "drivetrain"),
    new TalonFX(32, "drivetrain"),
    new TalonFX(42, "drivetrain")
  };

  // Drive motors
  public static TalonFX drive[] = 
  {
    new TalonFX(13, "drivetrain"),
    new TalonFX(23, "drivetrain"),
    new TalonFX(33, "drivetrain"),
    new TalonFX(43, "drivetrain")
  };
  
  // Default drive rotation directions
  // summer 2022 bot:  public static double[] defaultrotation = { -1, 1, 1, 1 };
  public static double[] defaultrotation = { -1, -1, -1, 1 };
  //public static double[] defaultrotation = new double[4];

  // Talon encoder pid controllers (TODO:  move these directly into the falcon 500's)
  public static PIDController talonpid[] =
  {
    new PIDController(0.015, 0.0, 0.0),
    new PIDController(0.015, 0.0, 0.0),
    new PIDController(0.015, 0.0, 0.0),
    new PIDController(0.015, 0.0, 0.0)
  };

  public double nearzero(double val)
  {
    if (val > -0.01 && val < 0.01) val = 0.0;
    return(val);
  }

  // Cancoder homing method
  // CancoderHome() reads the candoder positions and sets the falcon encoders accordingly
  public static void CancoderHome(double delay)
  {
    double x;
    System.out.println("Begin CancoderHome()");
    for(int lp=0; lp<4; lp++)
    {
      talonpid[lp].reset();
      x = (cancoder[lp].getAbsolutePosition() - cancoderoffset[lp]) / 180 * (Constants.talon_mk4i_360_count / 2);
      steer[lp].setSelectedSensorPosition(-1 * x, 0, 0);
      x = steer[lp].getSelectedSensorPosition(0);
    }
    Timer.delay(delay);     // Optional wait for CANbus
    System.out.println("End CancoderHome()");
  }

  public SwerveDriveSubsystem(ElevatorSubsystem elevator) 
  {
    this.elevator = elevator;

    gyro.calibrate();
    while(gyro.isCalibrating())
    {}
    
    // current limit drive falcons
    SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
    falconlimit.enable = true;
    falconlimit.currentLimit = 45;
    falconlimit.triggerThresholdCurrent = 45;
    falconlimit.triggerThresholdTime = 0;

    for(int lp=0; lp<4; lp++)
    {
      steer[lp].configFactoryDefault();
      steer[lp].setNeutralMode(NeutralMode.Brake);
      steer[lp].configSupplyCurrentLimit(falconlimit);

      drive[lp].configFactoryDefault();
      drive[lp].setNeutralMode(NeutralMode.Brake);
      drive[lp].configSupplyCurrentLimit(falconlimit);

      /*
      // per ctre velocity control example
      drive[lp].configNeutralDeadband(0.001);
      drive[lp].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      drive[lp].configNominalOutputForward(0, 30);
            drive[lp].configNominalOutputReverse(0, 30);
            drive[lp].configPeakOutputForward(1, 30);
            drive[lp].configPeakOutputReverse(-1, 30);

            drive[lp].config_kF(0, 1023.0 / 20660.0, 30);
            drive[lp].config_kP(0, 0.1, 30);
            drive[lp].config_kI(0, 0.001, 30);
            drive[lp].config_kD(0, 5, 30);    
      */

    }
    gyro.reset();
    gyro.zeroYaw();
    //SmartDashboard.putData(this);

    /*
    if(robotDigitalInput.get())
    {
      cancoderoffset[0] = 157;
      cancoderoffset[1] = 224;
      cancoderoffset[2] = 135;
      cancoderoffset[3] = 174;

      defaultrotation[0] = -1;
      defaultrotation[1] = -1;
      defaultrotation[2] = -1;
      defaultrotation[3] = -1;
    }
    else
    {
      cancoderoffset[0] = 170;
      cancoderoffset[1] = 228;
      cancoderoffset[2] = 170;
      cancoderoffset[3] = 204;

      defaultrotation[0] = -1;
      defaultrotation[1] = 1;
      defaultrotation[2] = 1;
      defaultrotation[3] = 1;
    }
    */
  }
  
  public void driveStrafe(double adjXj, double initialRobotYaw, double initialTicks)
  {
    double gyroError = Math.abs(initialRobotYaw) - Math.abs(gyroYaw);
    double kPMultiplier = 0;
    double adjZj = 0;

    if((Math.abs(initialTicks - drive[0].getSelectedSensorPosition(0))) >= 5000)
    {
      if(gyroYaw < 0 )
      {
        kPMultiplier = -1;
      }
      else
      {
        kPMultiplier = 1;
      }

      adjZj = gyroError * ((kPMultiplier) * 0.02);    //0.015
    }

    if(elevator.elevatorActive())
    {
      adjXj = 0.375 * adjXj;
      adjZj = 0.375 * adjZj;
    }

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
    double deadband = 0;  // Originally 0.075
    double STR = adjXj; 
    if (STR > -deadband && STR < deadband) STR = 0;

    double rotationDeadband = 0;
    double RCW = adjZj;
    if (RCW > -rotationDeadband && RCW < rotationDeadband) RCW = 0;

    RCW /= 3;

    double FWD = 0;
    
    // adjust for field oriented drive
    //double gyro_rad = gyroYaw / 180 * Math.PI;
    double gyro_rad = (gyroYaw + 180) / 180 * Math.PI;
    double tFWD = STR * Math.sin(gyro_rad);
    STR = STR * Math.cos(gyro_rad);
    FWD = tFWD;

    //SmartDashboard.putNumber("aSTR", STR);
    //SmartDashboard.putNumber("aFWD", FWD);
    
    // Compute temporary work variables
    double A = nearzero(STR + RCW * (L/R));
    double B = nearzero(STR - RCW * (L/R));
    double C = nearzero(FWD + RCW * (W/R));
    double D = nearzero(FWD - RCW * (W/R));

    // Compute angles (range is -180 to 180 degrees)
    double[] angle =
    {
      nearzero(Math.atan2(A,D) * 180 / Math.PI),
      nearzero(Math.atan2(A,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,D) * 180 / Math.PI)
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
      defaultrotation[0],
      defaultrotation[1],
      defaultrotation[2],
      defaultrotation[3]
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
        double ec = steer[lp].getSelectedSensorPosition(0);
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
          rotation[lp] = -defaultrotation[lp];
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

        co = MathUtil.clamp(talonpid[lp].calculate(pv, sp), MinSteer, MaxSteer);

        // Send steering output to falcon
        steer[lp].set(ControlMode.PercentOutput, co);

        // Send drive output to falcon
        drive[lp].set(ControlMode.PercentOutput, rotation[lp] * speed[lp]);

      }
    }
  }

  public void driveStraight(double adjYj, double initialRobotYaw, double initialTicks)
  {
    double gyroError = Math.abs(initialRobotYaw) - Math.abs(gyroYaw);
    double kPMultiplier = 0;
    double adjZj = 0;

    if((Math.abs(initialTicks - drive[0].getSelectedSensorPosition(0))) >= 5000)
    {
      if(gyroYaw < 0 )
      {
        kPMultiplier = -1;
      }
      else
      {
        kPMultiplier = 1;
      }

      adjZj = gyroError * ((kPMultiplier) * 0.03);    //0.015
    }

    if(elevator.elevatorActive())
    {
      adjYj = 0.375 * adjYj;
      adjZj = 0.375 * adjZj;
    }

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

    double FWD = adjYj;
    if (FWD > -deadband && FWD < deadband) FWD = 0;

    double rotationDeadband = 0.0125;
    double RCW = adjZj;
    if (RCW > -rotationDeadband && RCW < rotationDeadband) RCW = 0;

    RCW /= 3;

    double STR = 0;

    // adjust for field oriented drive
    //double gyro_rad = gyroYaw / 180 * Math.PI;
    double gyro_rad = (gyroYaw + 180) / 180 * Math.PI;
    double tFWD = FWD * Math.cos(gyro_rad);
    STR = -FWD * Math.sin(gyro_rad);
    FWD = tFWD;

    // Compute temporary work variables
    double A = nearzero(STR + RCW * (L/R));
    double B = nearzero(STR - RCW * (L/R));
    double C = nearzero(FWD + RCW * (W/R));
    double D = nearzero(FWD - RCW * (W/R));

    // Compute angles (range is -180 to 180 degrees)
    double[] angle =
    {
      nearzero(Math.atan2(A,D) * 180 / Math.PI),
      nearzero(Math.atan2(A,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,D) * 180 / Math.PI)
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
      defaultrotation[0],
      defaultrotation[1],
      defaultrotation[2],
      defaultrotation[3]
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
        double ec = steer[lp].getSelectedSensorPosition(0);
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
          rotation[lp] = -defaultrotation[lp];
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

        co = MathUtil.clamp(talonpid[lp].calculate(pv, sp), MinSteer, MaxSteer);

        // Send steering output to falcon
        steer[lp].set(ControlMode.PercentOutput, co);

        // Send drive output to falcon
        drive[lp].set(ControlMode.PercentOutput, rotation[lp] * speed[lp]);

      }
    }

    // Otherwise stop motors
    else
    {
      for(int lp=0; lp<4; lp++)
      {
        steer[lp].set(ControlMode.PercentOutput, 0);
        drive[lp].set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public void driveSpline(double adjXj, double adjYj, double initialTicks, double desiredAngle)
  {
    double desiredRobotYaw;
    double kP;

    double adjZj = 0;
    double gyroError;


    desiredRobotYaw = desiredAngle;
    kP = 0.01;


    gyroError = desiredRobotYaw - gyroYaw;

    /*
    if((Math.abs(initialTicks - drive[0].getSelectedSensorPosition(0))) >= 5000)
    {
      if(gyroYaw < 0 )
      {
        kPMultiplier = -1;
      }
      else
      {
        kPMultiplier = 1;
      }
      */

      adjZj = MathUtil.clamp(gyroError *  kP, -.50, 0.50);    //0.015

    /*
    }
    else
    {
      adjZj = 0;
    }
    */

    if(elevator.elevatorActive())
    {
      adjXj = 0.375 * adjXj;
      adjYj = 0.375 * adjYj;
      adjZj = 0.375 * adjZj;
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
    double deadband = 0;  // Originally 0.075
    double STR = adjXj; 
    if (STR > -deadband && STR < deadband) STR = 0;

    double FWD = adjYj;
    if (FWD > -deadband && FWD < deadband) FWD = 0;

    double rotationDeadband = 0.0125;
    double RCW = adjZj;
    if (RCW > -rotationDeadband && RCW < rotationDeadband) RCW = 0;

    RCW /= 2;

    // adjust for field oriented drive
    //double gyro_rad = gyroYaw / 180 * Math.PI;
    double gyro_rad = (gyroYaw + 180) / 180 * Math.PI;
    double tFWD = FWD * Math.cos(gyro_rad) + STR * Math.sin(gyro_rad);
    STR = -FWD * Math.sin(gyro_rad) + STR * Math.cos(gyro_rad);
    FWD = tFWD;

    System.out.printf("%10.2f %10.2f %10.2f \n", FWD, STR, RCW);

    // Compute temporary work variables
    double A = nearzero(STR + RCW * (L/R));
    double B = nearzero(STR - RCW * (L/R));
    double C = nearzero(FWD + RCW * (W/R));
    double D = nearzero(FWD - RCW * (W/R));

    // Compute angles (range is -180 to 180 degrees)
    double[] angle =
    {
      nearzero(Math.atan2(A,D) * 180 / Math.PI),
      nearzero(Math.atan2(A,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,D) * 180 / Math.PI)
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
      defaultrotation[0],
      defaultrotation[1],
      defaultrotation[2],
      defaultrotation[3]
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
        double ec = steer[lp].getSelectedSensorPosition(0);
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
          rotation[lp] = -defaultrotation[lp];
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

        co = MathUtil.clamp(talonpid[lp].calculate(pv, sp), MinSteer, MaxSteer);

        // Send steering output to falcon
        steer[lp].set(ControlMode.PercentOutput, co);

        // Send drive output to falcon
        drive[lp].set(ControlMode.PercentOutput, rotation[lp] * speed[lp]);

      }
    }

    // Otherwise stop motors
    else
    {
      for(int lp=0; lp<4; lp++)
      {
        steer[lp].set(ControlMode.PercentOutput, 0);
        drive[lp].set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public void rotateInPlace(double desiredAngle)
  {
    double desiredRobotYaw;
    double kP;

    double adjZj = 0;
    double gyroError;


    desiredRobotYaw = desiredAngle;
    kP = 0.015;

    if(gyroYaw < 0)
    {
      gyroError = desiredRobotYaw - gyroYaw;
    }
    else
    {
      gyroError = desiredRobotYaw - gyroYaw;
    }

    /*
    if((Math.abs(initialTicks - drive[0].getSelectedSensorPosition(0))) >= 5000)
    {
      if(gyroYaw < 0 )
      {
        kPMultiplier = -1;
      }
      else
      {
        kPMultiplier = 1;
      }
      */

      adjZj = gyroError *  kP;    //0.015

    /*
    }
    else
    {
      adjZj = 0;
    }
    */

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


    double rotationDeadband = 0.0125;
    double RCW = adjZj;
    if (RCW > -rotationDeadband && RCW < rotationDeadband) RCW = 0;

    RCW /= 2;

    // Compute temporary work variables
    double A = nearzero(RCW * (L/R));
    double B = nearzero(-RCW * (L/R));
    double C = nearzero(RCW * (W/R));
    double D = nearzero(-RCW * (W/R));

    // Compute angles (range is -180 to 180 degrees)
    double[] angle =
    {
      nearzero(Math.atan2(A,D) * 180 / Math.PI),
      nearzero(Math.atan2(A,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,D) * 180 / Math.PI)
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
      defaultrotation[0],
      defaultrotation[1],
      defaultrotation[2],
      defaultrotation[3]
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
        double ec = steer[lp].getSelectedSensorPosition(0);
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
          rotation[lp] = -defaultrotation[lp];
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

        co = MathUtil.clamp(talonpid[lp].calculate(pv, sp), MinSteer, MaxSteer);

        // Send steering output to falcon
        steer[lp].set(ControlMode.PercentOutput, co);

        // Send drive output to falcon
        drive[lp].set(ControlMode.PercentOutput, rotation[lp] * speed[lp]);

      }
    }

    // Otherwise stop motors
    else
    {
      for(int lp=0; lp<4; lp++)
      {
        steer[lp].set(ControlMode.PercentOutput, 0);
        drive[lp].set(ControlMode.PercentOutput, 0);
      }
    }
  }


  public void driveSwerve(double Xj, double Yj, double Zj, boolean fieldorient)
  {
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
    
    // Convert Xj, Yj, and Zj to strafe, forward, and rotate
    double STR = Xj;   
    double FWD = -Yj;
    double RCW = Zj;
    
    // Limit rotate speed
    RCW /= 2;
    
    // adjust for field oriented drive
    if (fieldorient == true)
    {
      double gyro_rad = (gyroYaw + 180) / 180 * Math.PI;
      double tFWD = FWD * Math.cos(gyro_rad) + STR * Math.sin(gyro_rad);
      STR = -FWD * Math.sin(gyro_rad) + STR * Math.cos(gyro_rad);
      FWD = tFWD;
    }
    
    // Compute temporary work variables
    double A = nearzero(STR + RCW * (L/R));
    double B = nearzero(STR - RCW * (L/R));
    double C = nearzero(FWD + RCW * (W/R));
    double D = nearzero(FWD - RCW * (W/R));
    
    // Compute angles (range is -180 to 180 degrees)
    double[] angle =
    {
      nearzero(Math.atan2(A,D) * 180 / Math.PI),
      nearzero(Math.atan2(A,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,C) * 180 / Math.PI),
      nearzero(Math.atan2(B,D) * 180 / Math.PI)
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
      defaultrotation[0],
      defaultrotation[1],
      defaultrotation[2],
      defaultrotation[3]
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
        double ec = steer[lp].getSelectedSensorPosition(0);
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
          rotation[lp] = -defaultrotation[lp];
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
    
        co = MathUtil.clamp(talonpid[lp].calculate(pv, sp), MinSteer, MaxSteer);
    
        // Send steering output to falcon
        steer[lp].set(ControlMode.PercentOutput, co);
    
        // Send drive output to falcon
        drive[lp].set(ControlMode.PercentOutput, rotation[lp] * speed[lp]);
    
      }
    }
    
    // Otherwise stop motors
    else
    {
      for(int lp=0; lp<4; lp++)
      {
        steer[lp].set(ControlMode.PercentOutput, 0);
        drive[lp].set(ControlMode.PercentOutput, 0);
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


  @Override
  public void periodic() 
  {
    gyroYaw = gyro.getYaw();
    gyroPitch = gyro.getPitch();
    gyroAngle = gyro.getAngle();
    SmartDashboard.putNumber("YAW", gyroYaw);
    SmartDashboard.putNumber("ROLL", gyro.getRoll());
    SmartDashboard.putNumber("ANGLE", gyroAngle);
    SmartDashboard.putNumber("PITCH", gyro.getPitch());

    for(int lp=0; lp<4; lp++)
    {
      SmartDashboard.putNumber(lp +"-" + cancoderoffset[lp], cancoder[lp].getAbsolutePosition());
    }

//    System.out.println(drive[0].getSelectedSensorPosition());
  
  }

}
  