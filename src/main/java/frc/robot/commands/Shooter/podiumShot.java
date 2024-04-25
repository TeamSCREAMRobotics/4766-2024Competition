// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

//set shot on the podium that aligns based on color and driver station.
public class podiumShot extends Command {
  
  int DriverStall;
  Translation2d target;
  double rotTarget;
  Timer waitTime = new Timer();
  Pose2d tar;

  Swerve s_Swerve;
  Shooter s_Shooter;
  Pivot s_Pivot;
  /** Creates a new podiumShot. */
  public podiumShot(Swerve swerve, Shooter shooter, Pivot pivot) {
    s_Shooter = shooter;
    s_Swerve = swerve;
    s_Pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    addRequirements(s_Shooter);
    addRequirements(s_Pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var Alliance = DriverStation.getAlliance();
    DriverStall = DriverStation.getLocation().getAsInt();

    if(Alliance.get() == DriverStation.Alliance.Red){

      if(DriverStall == 1 || DriverStall == 2){
      target = new Translation2d(-0.05, 0);
      rotTarget = .275;
      }

      else if(DriverStall == 3){
        target = new Translation2d(-0.01, 0);
        rotTarget = .2;
      }
    
    }

    if(Alliance.get() == DriverStation.Alliance.Blue){
      if(DriverStall == 1 || DriverStall == 2){
      target = new Translation2d(0.05, 0);
      rotTarget = -.275;
      }

      else if(DriverStall == 3){
        target = new Translation2d(0.01, 0);
        rotTarget = -0.2;
      }

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.autoDrive(s_Swerve.robotRelativeSpeeds(target, rotTarget));
    
         // Sends pivot to setpoint which is the value fetched from the table.
        s_Pivot.goToSetPoint(2.1);

        // Makes sure that the pivot is not moving before starting the flywheels.
        if(s_Pivot.pivotMaster.getVelocity().getValueAsDouble() == 0 && s_Pivot.pivotMaster.getPosition().getValueAsDouble() > 2.1 - 0.005 && s_Pivot.pivotMaster.getPosition().getValueAsDouble() < 2.1 + 0.005){
          waitTime.start();
          s_Shooter.shoot(11);
        }

    
        
        //Checks to make sure that the flywheels are at the proper voltage before moving the note.
        if(s_Shooter.shooterMaster.getMotorVoltage().getValueAsDouble() > 10 - 0.05 && s_Shooter.shooterMaster.getVelocity().getValueAsDouble() > 10 * 5 && waitTime.get() > 0.5){
          s_Shooter.runConveyor(ShooterConstants.conveyorIntakeOutput);
        }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.resetConveyor();
    s_Shooter.resetShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_Shooter.beamBreakTriggered();
  }
}
