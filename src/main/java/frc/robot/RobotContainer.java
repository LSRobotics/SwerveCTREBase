// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ClearIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PassToShooterCmd;
import frc.robot.commands.RunIndexCommand;
import frc.robot.commands.ShooterRampUpCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import com.playingwithfusion.TimeOfFlight;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My joystick
  private final CommandXboxController operatorController = new CommandXboxController(1);
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

  //Our Code from 2024Robot
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  //private final WristSubsystem m_wrist = new WristSubsystem();
  private final LEDSubsystem m_leds = new LEDSubsystem();

  private TimeOfFlight indexBeamBreak = new TimeOfFlight(IndexerConstants.indexBeamBreakChannel);

  public static SendableChooser<Command> autoChooser;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    driverController.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverController.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    drivetrain.registerTelemetry(logger::telemeterize);


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
    //Our Code Pulled from 2024Robot
    //driverController.a().onTrue(Commands.parallel(new WristMovementCommand(()->2, m_wrist), new ShooterRampUpCommand(m_shooter, m_leds, .7)));
    //driverController.x().onTrue(new InstantCommand(() -> m_Blinkin.set(-0.87)));
    driverController.b().onTrue(new IntakeCommand(m_intake, m_indexer, m_leds, IntakeConstants.intakeSpeed, IndexerConstants.indexSpeed, null));
    driverController.a().whileTrue(new ClearIntakeCommand(m_intake, m_indexer, IntakeConstants.intakeSpeed, IndexerConstants.indexSpeed));


    //operatorController.leftTrigger().whileTrue(m_indexer, IntakeConstants.intakeSpeed);
    operatorController.rightTrigger().whileTrue(new RunIndexCommand(m_indexer, IndexerConstants.indexSpeed));
    operatorController.leftTrigger().whileTrue(new RunIndexCommand(m_indexer, -IndexerConstants.indexSpeed));
        


    //operatorController.povUp().onTrue(new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true));
    //operatorController.povDown().onTrue(new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, false));
    //operatorController.b().onTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_indexer, m_leds, 0.6),
    //                                                //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true),
    //                                            new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
    //operatorController.a().whileTrue(Commands.parallel(new ShooterRampUpCommand(m_shooter, m_leds, ShooterConstants.distanceShotSpeed, null),
      //new ElevatorToSetPointCmd(m_elevator, m_leds, ElevatorConstants.elevatorSpeed, true),
      //new WristMovementCommand(()-> WristConstants.distanceAngle, m_wrist)));
    //operatorController.rightTrigger().onTrue(new PassToShooterCmd(m_indexer, m_leds, 0.6));

    
  }

  public RobotContainer() {
    NamedCommands.registerCommand("ShooterRampUp", new ShooterRampUpCommand(m_shooter, m_leds, ShooterConstants.distanceShotSpeed, () -> notePresent()));
    NamedCommands.registerCommand("Intake", new IntakeCommand(m_intake, m_indexer, m_leds, IntakeConstants.intakeSpeed, IndexerConstants.indexSpeed, () -> notePresent()));
    NamedCommands.registerCommand("PassToShooter", new PassToShooterCmd(m_indexer, IndexerConstants.indexSpeed, () -> notePresent()));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);

        
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public boolean notePresent() {
    System.out.println(" " + indexBeamBreak.getRange()); //TODO: delete after testing
    return indexBeamBreak.getRange() <= IndexerConstants.beamBreakRange;
  }
}