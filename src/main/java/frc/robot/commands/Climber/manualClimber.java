// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class manualClimber extends Command {
  Climber s_Climber = new Climber();
  DoubleSupplier leftY;
  /** Creates a new manualClimber. */
  public manualClimber(Climber climber, DoubleSupplier lefty) {
    s_Climber = climber;
    leftY = lefty;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }


  // Called when the command is initially scheduled.


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double percent = leftY.getAsDouble();
    s_Climber.manualClimb(percent);

    //System.out.println(s_Climber.climbPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
