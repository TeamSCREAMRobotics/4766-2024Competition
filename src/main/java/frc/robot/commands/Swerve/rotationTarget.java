// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class rotationTarget extends Command {
  Swerve s_Swerve;
  double rotationTarget;
  /** Creates a new rotationTarget. */
  public rotationTarget(Swerve swerve) {
    s_Swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationTarget = 30;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.autoDrive(s_Swerve.fieldRelativeSpeeds(new Translation2d(), 30));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Swerve.gyro.getYaw().getValueAsDouble()>=rotationTarget;
  }
}
