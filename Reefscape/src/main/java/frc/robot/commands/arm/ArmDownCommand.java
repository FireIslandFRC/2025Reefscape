package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDownCommand extends Command{

    public ArmDownCommand() {
      }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubsystem.armDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
