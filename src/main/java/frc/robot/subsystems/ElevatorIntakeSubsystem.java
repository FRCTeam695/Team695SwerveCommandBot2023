// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorIntakeSubsystem extends SubsystemBase 
{
  private CANSparkMax m_intake = new CANSparkMax(9, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_intake.getEncoder();

  private Timer stalltimer = new Timer();
  private double lastposition;
  private boolean stallhold;
  private boolean running;
  private double runspeed;

  public ElevatorIntakeSubsystem() 
  {
    m_intake.restoreFactoryDefaults();
    m_intake.setIdleMode(IdleMode.kBrake);
    m_encoder.setPosition(0);
    stallhold = false;
    running = false;
    NetworkTableInstance.getDefault().getTable("sidecar695").getIntegerTopic("stalled").publish().set(0);
  }

  // direction:  -1=in, 1=out
  public void runInit(double direction)
  {
    // if in request and already running or stalled, stop motor
    if (direction == -1)
    {

      if ((running == true) || (stallhold == true))
      {
        running = false;
        runspeed = 0;
        m_intake.set(runspeed);
        stallhold = false;
        NetworkTableInstance.getDefault().getTable("sidecar695").getIntegerTopic("stalled").publish().set(0);
        return;
      }
    }

    // stop motor if existing stall hold
    if (stallhold == true)
    {
      runspeed = 0;
      m_intake.set(runspeed);
      stallhold = false;
      NetworkTableInstance.getDefault().getTable("sidecar695").getIntegerTopic("stalled").publish().set(0);
    }

    // read current encoder for stall detection
    lastposition = m_encoder.getPosition();

    // reset stall timer
    stalltimer.reset();
    stalltimer.start();

    // get game piece type being played to set intake direction and current limits
    long currentIntakeMode = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentIntakeMode").getInteger(-1);

    // cones
    if (currentIntakeMode == 1)
    {
      m_intake.setSmartCurrentLimit(30);
    }

    // cubes
    else
    {
      // in
      if (direction == -1)
      {
        m_intake.setSmartCurrentLimit(25);
      }
      //out
      else
      {
        m_intake.setSmartCurrentLimit(25);
      }

      // direction must be inverted for cubes
      direction *= -1;
    }

    // set runspeed to max depending on direction
    if (direction == -1)
    {
      runspeed = -1;
    }

    else
    {
      runspeed = 1;
    }

    running = true;
    m_intake.set(runspeed);

  }

  public void stop()
  {
    running = false;
    if (stallhold == false)
    {
      runspeed = 0;
      m_intake.set(runspeed);
    }
  }

  @Override
  public void periodic() 
  {
    double currentposition;

    if (running == true)
    {
      if (stalltimer.get() > 0.250)
      {
        currentposition = m_encoder.getPosition();
        if (Math.abs(currentposition - lastposition) < 3)
        {
          m_intake.setSmartCurrentLimit(3);
          stallhold = true;
          NetworkTableInstance.getDefault().getTable("sidecar695").getIntegerTopic("stalled").publish().set(1);
        }
        else
        {
          lastposition = currentposition;
        }
        stalltimer.reset();
        stalltimer.start();
      }
    }

  }

}
