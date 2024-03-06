// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;



import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.tagLib;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.LimelightHelper;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class Target extends Command {
  Swerve s_Swerve;
  Limelight s_Limelight;
  PIDController xController;
  PIDController yController;
  PIDController rotController;
  tagLib TagLib;
  int targetTag;

  //Target is activated when the B button is pushed and ends when it gets within a certain tolerance
  public Target(Swerve s_Swerve, Limelight s_Limelight, ScreamPIDConstants translationXConstants, ScreamPIDConstants translationYConstants, ScreamPIDConstants rotationConstants) {
    this.s_Swerve = s_Swerve;
    this.s_Limelight = s_Limelight;
    xController = translationXConstants.toPIDController();
    yController = translationYConstants.toPIDController();
    rotController = rotationConstants.toPIDController();
    TagLib = new tagLib();
    

    xController.setTolerance(0.5);
    yController.setTolerance(0.5);
    rotController.setTolerance(0.2);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    addRequirements(s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Limelight.setPipeline(0);
    double currentTag = LimelightHelper.getFiducialID(getName());
    targetTag = TagLib.getTagNumber(currentTag);
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double checking if the tag is in the expected range (if not code will never run)
    //If you see this message there is something seriously wrong with tagLib (womp womp)
    if(targetTag <=0||targetTag>=7){
      System.out.println("Error - Invalid Tag ID: " + targetTag);
      return;
    }
    //Setpoint is the target value, where you want the robot to go.
    double xValue = LimelightHelper.getTV("limelight") ? -yController.calculate(LimelightHelper.getTY("limelight"), TagLib.yVals[targetTag]) : 0;
    double yValue = LimelightHelper.getTV("limelight") ? xController.calculate(LimelightHelper.getTX("limelight"), TagLib.xVals[targetTag]) : 0;
    if(LimelightHelper.getTY("limelight")>-0.2&&LimelightHelper.getTY("limelight")<0.2) xValue = 0;
    
    double rotValue =/*  LimelightHelper.getTV("limelight") ?  */rotController.calculate(LimelightHelper.getTX("limelight"), 0);
    
    
    //drives the robot to the setpoint as determined by the PIDControllers
    s_Swerve.autoDrive(
      s_Swerve.robotRelativeSpeeds(new Translation2d(xValue, yValue), rotValue)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.autoDrive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint()) || TagLib.IDNotFound();
  }
}