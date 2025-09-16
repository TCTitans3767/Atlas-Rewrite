package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotControl.RobotControl;

public class SetModeCommand extends Command{
    
    Command newMode;

    public SetModeCommand(Command newMode) {
        this.newMode = newMode;
    }

    @Override
    public void initialize() {
        RobotControl.setCurrentMode(newMode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
