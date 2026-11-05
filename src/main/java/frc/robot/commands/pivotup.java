package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot;

public class pivotup extends Command{
    private final pivot m_pivot;
    
    public pivotup(pivot ind) {
        m_pivot = ind;
        addRequirements(m_pivot);
    }

     @Override
     public void initialize(){}


    @Override
    public void execute() {
         m_pivot.moveToAngleCommand(45);
    }


    @Override
    public void end(boolean interrupted) {
     m_pivot.stopCommand();
}



    @Override
    public boolean isFinished() {
        return false;
    }
}