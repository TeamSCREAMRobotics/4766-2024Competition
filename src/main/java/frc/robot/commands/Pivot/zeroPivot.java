// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class zeroPivot extends Command {
  Pivot s_Pivot = new Pivot();
  /** Creates a new zeroPivot. */
  public zeroPivot(Pivot pivot) {
    s_Pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Pivot);
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pivot.setZero();
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
