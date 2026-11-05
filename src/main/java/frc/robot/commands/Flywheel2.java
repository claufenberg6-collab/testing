package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel2;

public class Flywheel2 extends Command{
    private final flywheel2 m_flywheel2;

    public Flywheel2(flywheel2 ind) {
        m_flywheel2 = ind; 
        addRequirements(m_flywheel2);
    }

    @Override 
public void initialize(){}

    @Override
public void execute() {
        m_flywheel2.spinUpCommandf2(-45);
    }

     @Override
 public void end(boolean interrupted) {
      m_flywheel2.stopCommandf2();
 }

     @Override
 public boolean isFinished() {
     return false;
 }
}