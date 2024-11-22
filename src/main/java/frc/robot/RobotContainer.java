// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ClearIntakeCmd;
import frc.robot.commands.ElevatorToSetPointCmd;
import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.PassToShooterCmd;
import frc.robot.commands.RunIndexCommand;
import frc.robot.commands.ShooterRampUpCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final LEDSubsystem m_leds = new LEDSubsystem();

  public static SendableChooser<Command> autoChooser;

  private TimeOfFlight indexBeamBreak = new TimeOfFlight(IndexerConstants.indexBeamBreakChannel);

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    drivetrain.registerTelemetry(logger::telemeterize);


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //driverController.a().onTrue(Commands.parallel(new WristMovementCommand(()->2, m_wrist), new ShooterRampUpCommand(m_shooter, m_leds, .7)));
        //driverController.x().onTrue(new InstantCommand(() -> m_Blinkin.set(-0.87)));
        //driverController.b().onTrue(new IntakeRunCommand(m_intake, m_indexer, m_leds, IntakeConstants.intakeSpeed, IndexerConstants.indexSpeed, () -> notePresent()));
        //driverController.a().whileTrue(new ClearIntakeCmd(m_intake, m_indexer, IntakeConstants.intakeSpeed, IndexerConstants.indexSpeed));


        //operatorController.leftTrigger().whileTrue(m_indexer, IntakeConstants.intakeSpeed);
        operatorController.rightTrigger().whileTrue(new RunIndexCommand(m_indexer, IndexerConstants.indexSpeed));
        operatorController.leftTrigger().whileTrue(new RunIndexCommand(m_indexer, -IndexerConstants.indexSpeed));
        


        operatorController.povUp().onTrue(new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true));
        operatorController.povDown().onTrue(new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, false));
        //operatorController.b().onTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_indexer, m_leds, 0.6),
        //                                                //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true),
            //                                            new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
        operatorController.a().whileTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_leds, ShooterConstants.distanceShotSpeed, ShooterConstants.distanceShotSpeed, null)));
                                                        //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true),
                                                        //new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
                                                                    
        

       // operatorController.b().whileTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_leds, 0.2, 0.2, null),
                                                        //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true),
        //                                                new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
        operatorController.x().whileTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_leds, -0.1, -0.1, null)));
                                                        //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true),
                                                        //new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
        //operatorController.rightTrigger().onTrue(new PassToShooterCmd(m_indexer, m_leds, 0.6));
         operatorController.y().whileTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_leds, ShooterConstants.distanceShotSpeed + 0.15, ShooterConstants.distanceShotSpeed + 0.15, null)));
                                                        //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed,s true),
                                                        //new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
    
        operatorController.b().whileTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_leds, 0.025, 0.265, null)));
                                                        //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed,s true),
                                                        //new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
 /*        driverController.leftTrigger().whileTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_leds, 0.45, 0.19, null),
    long shot                                           //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed,s true),
                                                        new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));         */                                       
        //driverController.rightTrigger().whileTrue(new RunIndexCommand(m_indexer, IndexerConstants.indexSpeed));

        NamedCommands.registerCommand("ShooterRampUp", new ShooterRampUpCommand(m_shooter, m_leds, ShooterConstants.distanceShotSpeed, ShooterConstants.distanceShotSpeed, () -> notePresent()));
        NamedCommands.registerCommand("Intake", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.53, 0.2, () -> notePresent()));
        NamedCommands.registerCommand("PassToShooter", new PassToShooterCmd(m_indexer, IndexerConstants.indexSpeed, () -> notePresent()));
        NamedCommands.registerCommand("IntakeStage", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.65, 0.255, () -> notePresent()));
        //NamedCommands.registerCommand("ShooterRampDown", new ShooterRampUpCommand(m_shooter, m_leds, -0.1, () -> !notePresent()));
        NamedCommands.registerCommand("IntakeFarNote", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.6, 0.24, () -> notePresent())); 
        NamedCommands.registerCommand("IntakeRedStage", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.43, 0.14, () -> notePresent()));

        NamedCommands.registerCommand("IntakeFourNote", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.5, 0.24, () -> notePresent()));
        NamedCommands.registerCommand("IntakeFourNoteTest", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.53, 0.21, () -> notePresent()));


        NamedCommands.registerCommand("ShooterRampDown", new ShooterRampUpCommand(m_shooter, m_leds, -0.3, - 0.3, () -> notePresent()));

        NamedCommands.registerCommand("IntakeStageTest", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.6, 0.24, () -> notePresent())); 

        NamedCommands.registerCommand("preLehighIntake", new IntakeRunCommand(m_intake, m_indexer, m_leds, 0.6, 0.2, () -> notePresent())); 
    } // TODO connect to april tags

  

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    configureBindings();
  }

  public boolean notePresent() {
    System.out.println(" " + indexBeamBreak.getRange()); //TODO: delete after testing
    return indexBeamBreak.getRange() <= IndexerConstants.beamBreakRange;
  }
  
  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}