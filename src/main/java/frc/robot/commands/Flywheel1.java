package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel1;

public class Flywheel1 extends Command{
    private final flywheel1 m_flywheel;

    public Flywheel1(flywheel1 ind) {
        m_flywheel = ind; 
        addRequirements(m_flywheel);
    }

    @Override 
public void initialize(){}

    @Override
public void execute() {
        m_flywheel.spinUpCommandf1(45);
    }

     @Override
 public void end(boolean interrupted) {
      m_flywheel.stopCommandf1();
 }

     @Override
 public boolean isFinished() {
     return false;
 }
}