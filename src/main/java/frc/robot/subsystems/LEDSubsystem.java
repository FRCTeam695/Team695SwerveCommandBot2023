// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase 
{
  private AddressableLED m_LED = new AddressableLED(0);
  private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(24);
  private String color;
  /** Creates a new LEDSubsystem. */
  
  public LEDSubsystem() 
  {
    color = "Off";
    m_LED.setLength(m_LEDBuffer.getLength());
  }

  public String getColor()
  {
    return color;
  }

  public void setColor(String newColor)
  {
    if( newColor.equals("Purple"))
    {
      for(int i = 0; i < m_LEDBuffer.getLength(); i++)
      {
        m_LEDBuffer.setRGB(i, 128, 0, 128);
      }
      color = newColor;
    }
    else if(newColor.equals("Yellow"))
    {
      for(int i = 0; i < m_LEDBuffer.getLength(); i++)
      {  
        m_LEDBuffer.setRGB(i, 255, 255, 0);
      }
      color = newColor;
    }
    else if(newColor.equals("Green"))
    {
      for(int i = 0; i < m_LEDBuffer.getLength(); i++)
      {  
        m_LEDBuffer.setRGB(i, 0, 255, 0);
      }
      color = newColor;
    }
    else if(newColor.equals("Off"))
    {
      for(int i = 0; i < m_LEDBuffer.getLength(); i++)
      {
        m_LEDBuffer.setRGB(i, 0, 0, 0);
      }
      color = newColor;
    }

    m_LED.setData(m_LEDBuffer);
    m_LED.start();
  }

  @Override
  public void periodic() 
  {
    long currentIntakeMode = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentIntakeMode").getInteger(-1);

    if (NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("stalled").getInteger(-1) == 1)
    {
      setColor("Green");
      return;
    }

    if(currentIntakeMode == 1)
    {
      setColor("Yellow");
    }
    else if (currentIntakeMode == 2)
    {
      setColor("Purple");
    }
    else
    {
      setColor("Off");
    }
  }
}

