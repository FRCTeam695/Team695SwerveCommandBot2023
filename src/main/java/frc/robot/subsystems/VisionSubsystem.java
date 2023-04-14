// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  PhotonCamera pv_camera = new PhotonCamera("OV5647");
  PhotonCamera life_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  boolean hasTargets;
  double pitch;
  double yaw;
  double area;

  double [] avg_pitch = {0, 0, 0, 0, 0};
  double [] avg_yaw = {0, 0, 0, 0, 0};
  double [] avg_area = {0, 0, 0, 0, 0};

  private int detectorPipeline;

  public VisionSubsystem() 
  {
  }

  /*public void getBestTarget()
  {
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();

    if(hasTargets)
    {
      var targlist = result.getTargets();
      var numTarg = targlist.size();
      target = result.getBestTarget();
      //target = targlist.get(0);
      //if (1 < numTarg)
      //{
      //  double bestarea = 0;
      //  double bestyaw = target.getYaw();
        //target = targ
      //}
    }
  }*/

  public void setPipeline(int pipeline)
  {
    detectorPipeline = pipeline;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(detectorPipeline);
  }

  public boolean hasTarget()
  {
    boolean ret = false;
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if(tv == 1)
    {
      ret = true;
    }
    return(ret);
  }

  // Methods for the game piece camera

  public boolean pv_hasTarget()
  {
    var result = pv_camera.getLatestResult();
    return(result.hasTargets());
  }

  public double pv_getYaw()
  {
    var result = pv_camera.getLatestResult();
    double ret = 0;
    if (result.hasTargets() == true)
    {
      ret = result.getBestTarget().getYaw();
    }
    else
    {
      ret = 0;
    }
    return(ret);
  }

  public double pv_getPitch()
  {
    var result = pv_camera.getLatestResult();
    double ret = 0;
    if (result.hasTargets() == true)
    {
      ret = result.getBestTarget().getPitch();
    }
    else
    {
      ret = 0;
    }
    return(ret);
  }

  public double pv_getArea()
  {
    var result = pv_camera.getLatestResult();
    double ret = 0;
    if (result.hasTargets() == true)
    {
      ret = result.getBestTarget().getArea();
    }
    else
    {
      ret = 0;
    }
    return(ret);
  }

  // Methods for the lifecam

  public boolean lifecam_hasTarget()
  {
    var result = life_camera.getLatestResult();
    return(result.hasTargets());
  }

  public double lifecam_getYaw()
  {
    var result = life_camera.getLatestResult();
    double ret = 0;
    if (result.hasTargets() == true)
    {
      ret = result.getBestTarget().getYaw();
    }
    else
    {
      ret = 0;
    }
    return(ret);
  }

  public double lifecam_getPitch()
  {
    var result = life_camera.getLatestResult();
    double ret = 0;
    if (result.hasTargets() == true)
    {
      ret = result.getBestTarget().getPitch();
    }
    else
    {
      ret = 0;
    }
    return(ret);
  }

  public double lifecam_getArea()
  {
    var result = life_camera.getLatestResult();
    double ret = 0;
    if (result.hasTargets() == true)
    {
      ret = result.getBestTarget().getArea();
    }
    else
    {
      ret = 0;
    }
    return(ret);
  }


  public double getPitch()
  {
    return pitch;
  }

  public double getYaw()
  {
    return yaw;
  }

  public double getArea()
  {
    return area;
  }

  @Override
  public void periodic() 
  {
    int lp;
    double tmp;


    boolean tg = hasTarget();

    if (tg == true)
    {

      tmp = 0;
      for(lp=0; lp<4; lp++)
      {
        avg_yaw[lp] = avg_yaw[lp+1];
        tmp += avg_yaw[lp];
      }
      avg_yaw[4] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      yaw = tmp / 5;

      tmp = 0;
      for(lp=0; lp<4; lp++)
      {
        avg_pitch[lp] = avg_pitch[lp+1];
        tmp += avg_pitch[lp];
      }
      avg_pitch[4] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      pitch = tmp / 5;

      tmp = 0;
      for(lp=0; lp<4; lp++)
      {
        avg_area[lp] = avg_area[lp+1];
        tmp += avg_area[lp];
      }
      avg_area[4] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
      area = tmp / 5;

    }
    else
    {
      yaw = 0;
      pitch = 0;
      area = 0;

      for(lp=5; lp<4; lp++)
      {
        avg_pitch[lp] = avg_yaw[lp] = avg_area[lp] = 0;
      }
    }

//    SmartDashboard.putBoolean("LT", tg);
    SmartDashboard.putNumber("LTPitch", pitch);
    SmartDashboard.putNumber("LTYaw", yaw);
//    SmartDashboard.putNumber("LTArea",area);  

  }
}
