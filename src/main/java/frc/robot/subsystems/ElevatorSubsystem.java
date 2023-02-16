// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase 
{
  private final WPI_TalonFX m_ElevatorFalcon = new WPI_TalonFX(2);
  
  private boolean HoldPos;
  private double TargetPos;
  private PIDController HoldPID = new PIDController(0.005, 0.001, 0);
  
  private double pos;

  // TODO:  set these level tick counts based on grid scoring posotions:
  private double[] levelTicks = {0, 30000, 60000, 87000};

  public ElevatorSubsystem() 
  {
//    m_ElevatorFalcon.setNeutralMode(NeutralMode.Brake);
    m_ElevatorFalcon.setNeutralMode(NeutralMode.Coast);

    m_ElevatorFalcon.configForwardSoftLimitThreshold(levelTicks[3]);
    m_ElevatorFalcon.configForwardSoftLimitEnable(true);
    m_ElevatorFalcon.configReverseSoftLimitThreshold(levelTicks[0]);
    m_ElevatorFalcon.configReverseSoftLimitEnable(true);

    SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
    falconlimit.enable = true;
    falconlimit.currentLimit = 50;
    falconlimit.triggerThresholdCurrent = 50;
    falconlimit.triggerThresholdTime = 0;

    TargetPos = 0;
    m_ElevatorFalcon.setSelectedSensorPosition(TargetPos,0,100);
    getPosition();
    HoldPos = true;
  }

  public void setSpeed(double percentVBus)
  {
    HoldPos = false;
    m_ElevatorFalcon.set(percentVBus);
  }

  public double getPosition()
  {
    pos = m_ElevatorFalcon.getSelectedSensorPosition();
    return pos;
  }

  public int getLevel()
  {
    pos = m_ElevatorFalcon.getSelectedSensorPosition();

    for(int i=3; i>0; i--)
    {
      if (pos >= levelTicks[i] - 1000)
      {
        return(i);
      }
      /*
      if (levelTicks[i]+1000>= pos && pos >= levelTicks[i]-1000)
      {
        return(i);
      }
      */
    }
    return(0);
  }

  public void runToLevel(int newLevel)
  {
    double maxTime = 2.0;     // maximum seconds to allow run
    double minSpeed = 0.2;    // start speed
    double maxSpeed = 0.6;   // plateau speed
    double plateauStart;      // threshold tick count when ramp up is complete
    double plateauEnd;        // threshold tick count to begin ramp down
    double targetSpeed;
    double ticks;

    int currentLevel = getLevel();

    // check for reset
    if (newLevel == -1)
    {
      m_ElevatorFalcon.setSelectedSensorPosition(0,0,100);
      return;
    }

    // if already at this level, do nothing
    if (newLevel == currentLevel)
    {
      return;
    }

    HoldPos = false;
    TargetPos = levelTicks[newLevel];

    // going up?
    if (newLevel > currentLevel)
    {

      // compute trapezoid inflection points
    //  plateauStart = levelTicks[currentLevel] + 0.2 * (levelTicks[newLevel] - levelTicks[currentLevel]);
  //   plateauEnd = levelTicks[currentLevel] + 0.8 * (levelTicks[newLevel] - levelTicks[currentLevel]);
      plateauStart = levelTicks[currentLevel] + 6000;
      plateauEnd = levelTicks[newLevel]-6000;
      // create a timer to enforce maxTime
      Timer et = new Timer();
      et.reset();
      et.start();

      // loop to run until maxTime
      while(et.get() < maxTime)
      {

        // get current encoder position
        ticks = getPosition();

        // if at the new level, exit
        if (ticks >= levelTicks[newLevel])
        {
          currentLevel = newLevel;
          break;
        }

        // before plateau speed ramp up
        if (ticks < plateauStart)
        {
          targetSpeed = maxSpeed * (ticks - levelTicks[currentLevel]) / (plateauStart - levelTicks[currentLevel]);
        }

        // at plateau speed constant at max
        else if (ticks < plateauEnd)
        {
          targetSpeed = maxSpeed;
        }

        // after plateau speed ramp down
        else
        {
          targetSpeed = maxSpeed - maxSpeed * (ticks - plateauEnd) / (levelTicks[newLevel] - plateauEnd);
        }

        // ensure target speed is at least at minimum (does not go to zero and stop prematurely)
        if (targetSpeed < minSpeed)
        {
          targetSpeed = minSpeed;
        }

        // send target speed to motor
        m_ElevatorFalcon.set(targetSpeed);

        // delay 5 msec to not saturate CAN
        Timer.delay(0.005);
      }

      m_ElevatorFalcon.set(0);

    }

    // going down?
    else
    {
      maxSpeed /= 2;

      // compute trapezoid inflection points
    //  plateauStart = levelTicks[currentLevel] - 0.2 * (levelTicks[currentLevel] - levelTicks[newLevel]);
    //  plateauEnd = levelTicks[currentLevel] - 0.8 * (levelTicks[currentLevel] - levelTicks[newLevel]);
      plateauStart = levelTicks[currentLevel] - 6000;
      plateauEnd = levelTicks[newLevel]+6000;
  
      // create a timer to enforce maxTime
      Timer et = new Timer();
      et.reset();
      et.start();

      // loop to run until maxTime
      while(et.get() < maxTime)
      {

        // get current encoder position
        ticks = getPosition();

        // if at the new level, exit
        if (ticks <= levelTicks[newLevel])
        {
          currentLevel = newLevel;
          break;
        }

        // before plateau speed ramp up
        if (ticks > plateauStart)
        {
          targetSpeed = -(maxSpeed * (ticks - levelTicks[currentLevel]) / (plateauStart - levelTicks[currentLevel]));
        }

        // at plateau speed constant at max
        else if (ticks > plateauEnd)
        {
          targetSpeed = -maxSpeed;
        }

        // after plateau speed ramp down
        else
        {
          targetSpeed = -(maxSpeed - maxSpeed * (ticks - plateauEnd) / (levelTicks[newLevel] - plateauEnd));
        }

        // ensure target speed is at least at minimum (does not go to zero and stop prematurely)
        if (targetSpeed > -minSpeed)
        {
          targetSpeed = -minSpeed;
        }

        // send target speed to motor
        m_ElevatorFalcon.set(targetSpeed);

        // delay 5 msec to not saturate CAN
        Timer.delay(0.005);
      }

      m_ElevatorFalcon.set(0);

    }

    HoldPos = true;

  }

  @Override
  public void periodic() 
  {
    double RPM = m_ElevatorFalcon.getSelectedSensorVelocity() * 600 / 2048;
    SmartDashboard.putNumber("RPM", RPM);
    
    SmartDashboard.putNumber("Elevator Position", pos);
    SmartDashboard.putNumber("Elevator Level", getLevel());
    if (HoldPos == true)
    {
      double co = MathUtil.clamp(HoldPID.calculate(getPosition(), TargetPos), -0.08, 0.08);
      if(pos < 500)
      {
        co = 0;
      }
      SmartDashboard.putNumber("CO", co);
      m_ElevatorFalcon.set(co);
    }
    else
    {
      HoldPID.reset();
    }
  }

}
