// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  double x;
  double y;
  double v; 
  double area;

  /** Creates a new Limelight. */
  public Limelight() {}

  public void setPipeline(int pipeline)
  {
        NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
        pipelineEntry.setNumber(pipeline);
  }

  public void update()
  {//read values periodically
      x = tx.getDouble(0.0);
      // System.out.println(area);
      y = ty.getDouble(0.0);
      v = tv.getDouble(0.0); 
      area = ta.getDouble(0.0);
  }

  public double getX(){
    update();
    return x;
  }

  public double getY(){
    update();
    return y;
  }

  public double getV(){
    update();
    return v;
  }

  public double getArea(){
    update();
    return area;
  }
}

  

