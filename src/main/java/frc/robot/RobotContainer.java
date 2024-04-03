package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ControllerRumble;
import frc.robot.commands.Climber.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Pivot.*;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final CommandXboxController commandOperator = new CommandXboxController(1);
    public final CommandXboxController commandDriver = new CommandXboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final Limelight s_Limelight = new Limelight();
    public final Intake s_Intake = new Intake();
    public final Shooter s_Shooter = new Shooter();
    public final Climber s_Climber = new Climber();
    public final Pivot s_Pivot = new Pivot();
   
    private SendableChooser<Command> auto;

    private boolean climbSetPoint = false;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //NamedCommands.registerCommand("Target", new Target(s_Swerve, s_Limelight, new ScreamPIDConstants(0.04,0,0.001), new ScreamPIDConstants(0.25, 0, 0.003), new ScreamPIDConstants(0.005,0,0)));
        NamedCommands.registerCommand("Amp", new armSetPoint(s_Pivot, s_Shooter, 6, ShooterConstants.shooterLowerVelocity));
        NamedCommands.registerCommand("Pivot Down", new sendPivotZero(s_Pivot, 0.5, 75));
        NamedCommands.registerCommand("Intake", new runIntake(s_Intake, s_Shooter, IntakeConstants.intakeOutput));
        NamedCommands.registerCommand("Shoot", new Shoot(s_Shooter, s_Pivot, ShooterConstants.shooterSpeakerVelocity));
        NamedCommands.registerCommand("ShootLow", new Shoot(s_Shooter, s_Pivot, ShooterConstants.shooterLowerVelocity));
<<<<<<< Updated upstream
=======
        NamedCommands.registerCommand("visionShotAuto", new visionShot(s_Swerve, s_Pivot, s_Shooter, s_Limelight));
>>>>>>> Stashed changes

        auto = new SendableChooser<Command>();
        //Basic Autos
        auto.setDefaultOption("Do Nothing", new PathPlannerAuto("Do Nothing"));
        auto.addOption("Go Forward", new PathPlannerAuto("Go Forward"));

        //Blue Speaker Only Autos
        auto.addOption("Amp Side Pivot 3 Note", new PathPlannerAuto("Blue 1 (SO)"));
        auto.addOption("4 Note", new PathPlannerAuto("Blue 2 (SO)"));

        //Red Speaker Only Autos
        auto.addOption("Amp Side Pivot 3 Note (Red)", new PathPlannerAuto("Red 1 (SO)"));
        
        //TODO:The Crease Needs to be 4inches from the edge of the subwoofer/speaker

        //Blue Main Autos
        auto.addOption("Amp Side Pivot 3 Note (Amp Score)", new PathPlannerAuto("Blue 1"));
        auto.addOption("Side 2 Note", new PathPlannerAuto("Blue 3"));
        auto.addOption("OffSide 2 Note", new PathPlannerAuto("Blue 4"));

        //Red Main Auto and MOVE
        auto.addOption("Amp Side Pivot 3 Note (Amp Score)(Red)", new PathPlannerAuto("Red 1"));
        auto.addOption("MOVE 1", new PathPlannerAuto("MOVE 1"));
        auto.addOption("MOVE 2", new PathPlannerAuto("MOVE 2"));
        auto.addOption("MOVE 3", new PathPlannerAuto("MOVE 3"));
        auto.addOption("Shoot", new PathPlannerAuto("Shoot"));
        auto.addOption("Blue Source Side", new PathPlannerAuto("Blue Source"));
        //Test
        
        

        
        SmartDashboard.putData(auto);
        HttpCamera limelight = new HttpCamera("limelight", "http://frcvision.local:1181/stream.mjpg");
        CameraServer.getVideo(limelight);
        Shuffleboard.getTab("SmartDashboard").add(limelight);
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                driver,
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> !robotCentric.getAsBoolean()
            )
        );

        s_Pivot.setDefaultCommand(new manualPivot(s_Pivot, () -> -commandOperator.getLeftY()/2));
        s_Climber.setDefaultCommand(new manualClimber(s_Climber, ()-> commandOperator.getRightY()));
       // s_Intake.setDefaultCommand(new resetIntake(s_Shooter, s_Intake));
        //s_Shooter.setDefaultCommand(new resetIntake(s_Shooter, s_Intake));
        

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //TODO: PID may need more fine tuning
       

        //Driver Controls
        //Right Trigger is already set to fast mode and Joysticks are already set up for swerve

<<<<<<< Updated upstream
=======
        commandDriver.a().toggleOnTrue(new visionShot(s_Swerve, s_Pivot, s_Shooter, s_Limelight));
>>>>>>> Stashed changes
        commandDriver.leftBumper().toggleOnTrue(new runIntake(s_Intake, s_Shooter, IntakeConstants.intakeOutput).andThen(new ControllerRumble(driver, 50)));
        commandDriver.rightBumper().onTrue(new Shoot(s_Shooter, s_Pivot, ShooterConstants.shooterSpeakerVelocity));
        commandDriver.leftTrigger(0.8).onTrue(new Outtake(s_Intake, s_Shooter, -0.5).alongWith(new sendPivotZero(s_Pivot, 2, 75)));
        

        commandDriver.b().onTrue(new InstantCommand(()-> s_Pivot.setZero()));
        
        //commandDriver.a().onTrue(new Target(s_Swerve, s_Limelight, new ScreamPIDConstants(0.04,0,0.001), new ScreamPIDConstants(0.25, 0, 0.003), new ScreamPIDConstants(0.005,0,0)));

        //Operator Controls
        //Left Joystick is already set to manualPivot.
        //Right Joystick is already set to manualClimb.
        commandOperator.povUp().onTrue(new armSetPoint(s_Pivot, s_Shooter, PivotConstants.ampSetPoint, ShooterConstants.shooterLowerVelocity).andThen(new sendPivotZero(s_Pivot, 2, 40)));
        commandOperator.povDown().onTrue(new armSetPoint(s_Pivot, s_Shooter, 2.1, ShooterConstants.shooterPodiumVelocity));
        commandOperator.povRight().onTrue(new InstantCommand(()-> s_Climber.setZero()));
      //Toggle up and down for climber
<<<<<<< Updated upstream
        commandOperator.povLeft().toggleOnTrue(new Climb(s_Climber, -140)).toggleOnFalse(new Climb(s_Climber, 0));
=======
        commandOperator.povLeft().toggleOnTrue(new Climb(s_Climber, -350)).toggleOnFalse(new Climb(s_Climber, 0));
        
>>>>>>> Stashed changes

    
        commandOperator.a().onTrue(new stopClimb(s_Climber));   
        commandOperator.b().onTrue(new zeroPivot(s_Pivot));
        commandOperator.x().onTrue(new resetIntake(s_Shooter, s_Intake));
      
           
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return auto.getSelected();
    }

}