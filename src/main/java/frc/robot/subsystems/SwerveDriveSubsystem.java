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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.*;

public class SwerveDriveSubsystem extends SubsystemBase 
{
  // Navx2 gyro
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Digital Input to determine robot
  public DigitalInput robotDigitalInput = new DigitalInput(9);

  public double gyroYaw;

  // Cancoders
  public static WPI_CANCoder cancoder[] =
  {
    new WPI_CANCoder(11),
    new WPI_CANCoder(21),
    new WPI_CANCoder(31),
    new WPI_CANCoder(41)
  };

  // Cancoder mounting orientation offsets (degrees)
 //static double[] cancoderoffset = { 170, 228, 170, 204 };
  static double[] cancoderoffset = { 157, 224, 135, 154 };
  //static double[] cancoderoffset = new double[4];

  // Steering motors
  public static TalonFX steer[] = 
  {
    new TalonFX(12),
    new TalonFX(22),
    new TalonFX(32),
    new TalonFX(42)
  };

  // Drive motors
  public static TalonFX drive[] = 
  {
    new TalonFX(13),
    new TalonFX(23),
    new TalonFX(33),
    new TalonFX(43)
  };
  
  // Default drive rotation directions (corner 1 is negated due to the camcoder mounting orientation)
  // public static double[] defaultrotation = { -1, 1, 1, 1 };
  public static double[] defaultrotation = { -1, -1, -1, -1 };
  //public static double[] defaultrotation = new double[4];


  // Cancoder pid controllers
  public static PIDController cancoderpid[] =
  {
    new PIDController(0.015, 0, 0.0001),
    new PIDController(0.015, 0, 0.0001),
    new PIDController(0.015, 0, 0.0001),
    new PIDController(0.015, 0, 0.0001)
  };

  // Talon encoder pid controllers (TODO:  move these directly into the falcon 500's)
  public static PIDController talonpid[] =
  {
    new PIDController(0.01, 0.001, 0),
    new PIDController(0.01, 0.001, 0),
    new PIDController(0.01, 0.001, 0),
    new PIDController(0.01, 0.001, 0)
  };

  public double nearzero(double val)
  {
    if (val > -0.01 && val < 0.01) val = 0.0;
    return(val);
  }

  // Cancoder homing method
  // CancoderHome() takes 1 second to home the swerve steer wheels using the cancoders
  public static void CancoderHome()
  {
    System.out.println("Begin CancoderHome()");
    for(int t=0; t<100; t++)
    {
      for(int lp=0; lp<4; lp++)
      {
        double co = MathUtil.clamp(cancoderpid[lp].calculate(cancoder[lp].getAbsolutePosition(), cancoderoffset[lp]), -1, 1);
        steer[lp].set(ControlMode.PercentOutput, -1 * co);
      }
      Timer.delay(0.01);
    }
    for(int lp=0; lp<4; lp++)
    {
      steer[lp].setSelectedSensorPosition(0);
      talonpid[lp].reset();
    }
    System.out.println("End CancoderHome()");
  }

  public SwerveDriveSubsystem() 
  {
    gyro.calibrate();
    while(gyro.isCalibrating())
    {}
    System.out.println("RUNNING SWERVE SUBSYSTEM CONSTRUCTOR");

    // current limit drive falcons
    SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
    falconlimit.enable = true;
    falconlimit.currentLimit = 20;
    falconlimit.triggerThresholdCurrent = 20;
    falconlimit.triggerThresholdTime = 0;

    for(int lp=0; lp<4; lp++)
    {
      steer[lp].configFactoryDefault();
      steer[lp].setNeutralMode(NeutralMode.Brake);
      steer[lp].configSupplyCurrentLimit(falconlimit);

      drive[lp].configFactoryDefault();
      drive[lp].setNeutralMode(NeutralMode.Brake);
      drive[lp].configSupplyCurrentLimit(falconlimit);

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
    }
    gyro.reset();
    SmartDashboard.putData(this);

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

  @Override
  public void periodic() 
  {
    gyroYaw = gyro.getYaw();
    SmartDashboard.putNumber("Yaw", gyroYaw);
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
