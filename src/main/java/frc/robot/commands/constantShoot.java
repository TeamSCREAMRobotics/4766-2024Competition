// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
//runs the flywheel constantly (useful when feeding so it takes less time to get them over)
public class constantShoot extends Command {
  Shooter s_Shooter;
  Intake s_Intake;
  int phase;
  /** Creates a new constantShoot. */
  public constantShoot(Shooter shooter, Intake intake) {
  s_Shooter = shooter;
  s_Intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
    addRequirements(s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.runIntake(IntakeConstants.intakeOutput);
    s_Shooter.shoot(3.3);

    if(phase == 0){
      s_Shooter.runConveyor(ShooterConstants.conveyorIntakeOutput);
    }

    if(s_Shooter.shooterMaster.getVelocity().getValueAsDouble() > 2.25 * 5 && phase == 1){
      s_Shooter.runConveyor(ShooterConstants.conveyorIntakeOutput);
    }

    if(s_Shooter.beamBreakTriggered()){
      phase = 1;
    }

    if(!s_Shooter.beamBreakTriggered() && phase == 1){
      phase = 2;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.resetIntake();
    s_Shooter.resetConveyor();
    s_Shooter.resetShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == 2;
  }
}
