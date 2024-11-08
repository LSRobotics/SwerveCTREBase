// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClearIntakeCmd extends Command {
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private double intakeSpeed;
  private double indexSpeed;

 
  public ClearIntakeCmd(IntakeSubsystem intake, IndexerSubsystem indexer, double intakeSpeed, double indexSpeed) {
    m_intake = intake;
    m_indexer = indexer;
    this.intakeSpeed = intakeSpeed;
    this.indexSpeed = indexSpeed;
    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runIntake(-intakeSpeed);
    m_indexer.runIndexer(-indexSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntake(0);
    m_indexer.runIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}