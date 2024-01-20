// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelper;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class Target extends Command {
  Swerve s_Swerve;
  Limelight s_Limelight;
  boolean atX = false;
  boolean atY = false;
  double translationDistance;
  double strafeDistance;
  double rotationDistance;
  double tagT;
  double tagS;
  double tagR;
  boolean atTarget = false;
  double validTarget;
  PIDController xController;
  PIDController yController;
  PIDController rotController;
  
  /** Creates a new Target. */
  public Target(Swerve s_Swerve, Limelight s_Limelight, ScreamPIDConstants translationXConstants, ScreamPIDConstants translationYConstants, ScreamPIDConstants rotationConstants) {
    this.s_Swerve = s_Swerve;
    this.s_Limelight = s_Limelight;
    xController = translationXConstants.toPIDController();
    yController = translationYConstants.toPIDController();
    rotController = rotationConstants.toPIDController();

    xController.setTolerance(1.2);
    yController.setTolerance(0.5);
    rotController.setTolerance(0.5);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    addRequirements(s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atTarget = false;
    s_Limelight.setPipeline(0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Limelight.getV()<1.0){
      atTarget = true;
      return;
    }
    double xValue = LimelightHelper.getTV("limelight") ? yController.calculate(LimelightHelper.getTY("limelight"), 0) : 0;
    double yValue = LimelightHelper.getTV("limelight") ? xController.calculate(LimelightHelper.getTX("limelight"), 0) : 0;
    double rotValue = rotController.calculate(s_Swerve.getGyroYaw().getDegrees(), 0);
     
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
    return atTarget;
  }
}
