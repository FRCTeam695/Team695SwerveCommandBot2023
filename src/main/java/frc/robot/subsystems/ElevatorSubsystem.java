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
import edu.wpi.first.networktables.NetworkTableInstance;

public class ElevatorSubsystem extends SubsystemBase 
{
  private final WPI_TalonFX m_ElevatorFalcon = new WPI_TalonFX(2);
  
  private boolean HoldPos;
  private double TargetPos;
  private PIDController HoldPID = new PIDController(0.005, 0.001, 0);
  
  private int currentLevel;
  private int newLevel;

  private double pos;

  private double[] levelTicks = {0, 10000, 54000, 80000, 86000};

  private Timer et = new Timer();

  public ElevatorSubsystem() 
  {
    m_ElevatorFalcon.setNeutralMode(NeutralMode.Coast);

    m_ElevatorFalcon.configForwardSoftLimitThreshold(levelTicks[4]);
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
    }
    return(0);
  }

  public void runToLevel()
  {
    long newLevel = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentLevel").getInteger(-1);
    
    // check for reset
    if (newLevel == -1)
    {
      m_ElevatorFalcon.setSelectedSensorPosition(0,0,100);
      return;
    }

    currentLevel = getLevel();

    // JPK - just toggle elevator movement between current level and ground floor
    if (currentLevel > 0)
    {
      newLevel = 0;
    }

    // if already at this level, do nothing
    if (newLevel == currentLevel)
    {
      return;
    }

    // kickoff elevator move in periodic callback
    this.newLevel = (int) newLevel;
    HoldPos = false;
    TargetPos = levelTicks[(int) newLevel];
    et.reset();
    et.start();
  }

  public boolean elevatorActive()
  {
    if (pos >= 1000)
    {
      return(true);
    }
    return(false);
  }

  @Override
  public void periodic() 
  {
    double maxTime = 3.0;     // maximum seconds to allow run
    double minSpeed = 0.2;    // start speed
    double maxSpeed = 1.0;      // plateau speed
    double plateauStart;      // threshold tick count when ramp up is complete
    double plateauEnd;        // threshold tick count to begin ramp down
    double targetSpeed;
    double ticks;

    SmartDashboard.putNumber("Elevator Position", pos);
    SmartDashboard.putNumber("Elevator Level", getLevel());

    if (HoldPos == true)
    {
      pos = getPosition();

      double co = MathUtil.clamp(HoldPID.calculate(pos, TargetPos), -0.08, 0.08);
      if(pos < 1000)
      {
        co = 0;
      }
      m_ElevatorFalcon.set(co);
      return;
    }

    // here if we're not holdoing position

    HoldPID.reset();
    if (newLevel == currentLevel)
    {
      HoldPos = true;
      return;
    }

    // check for move timeout
    if (et.get() >= maxTime)
    {
      HoldPos = true;
      return;
    }

    // going up?
    if (newLevel > currentLevel)
    {

      // compute trapezoid inflection points
      plateauStart = levelTicks[currentLevel];
      plateauEnd = levelTicks[newLevel];
        
      // get current encoder position
      ticks = getPosition();

      // if at the new level, exit
      if (ticks >= levelTicks[newLevel])
      {
        currentLevel = newLevel;
        HoldPos = true;
        m_ElevatorFalcon.set(0);
        return;
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

    }

    // going down?
    else
    {
      maxSpeed *= 0.4;

      // compute trapezoid inflection points
      plateauStart = levelTicks[currentLevel];
      plateauEnd = levelTicks[newLevel] + 6000;

      // get current encoder position
      ticks = getPosition();

      // if at the new level, exit
      if (ticks <= levelTicks[newLevel])
      {
        currentLevel = newLevel;
        HoldPos = true;
        return;
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

    }

  }

}

