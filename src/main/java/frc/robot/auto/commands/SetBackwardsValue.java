// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.manuals.Backwards;

public class SetBackwardsValue extends Command {

  double d;
  public SetBackwardsValue(double d){
    this.d = d;
  }

  @Override
  public void initialize() {
    Backwards.setDistance(d);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}