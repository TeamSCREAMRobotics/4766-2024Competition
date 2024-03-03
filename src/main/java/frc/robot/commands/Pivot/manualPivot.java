// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class manualPivot extends Command {
  Pivot s_Pivot = new Pivot();
  DoubleSupplier leftTrig;
  DoubleSupplier rightTrig;
  /** Creates a new manualPivot. */
  // doubleSupplier activly passes through numbers, hence why it works in this case
  public manualPivot(Pivot pivot, DoubleSupplier percent1) {
    s_Pivot = pivot;

    // leftTrig is currently just being passed through as the left joystick on the operator.
    leftTrig = percent1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Pivot);
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double percent = leftTrig.getAsDouble();
    s_Pivot.manualPivot(percent);
    

  }

  
}
