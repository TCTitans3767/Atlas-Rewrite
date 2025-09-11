package frc.robot.subsystems.robotControl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RobotControl extends SubsystemBase implements RobotControlIO{

    private final RobotControlIOInputsAutoLogged inputs = new RobotControlIOInputsAutoLogged();
    private final RobotControlIO io = this;

    private static Command currentCommand = Commands.none();
    private static Command currentDriveCommand = Commands.none();
    private static Command previousCommand = Commands.none();
    private static Command previousDriveCommand = Commands.none();

    public RobotControl() {
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("RobotControl", inputs);
    }

    @Override
    public void updateInputs(RobotControlIOInputs inputs) {
       inputs.CurrentCommand = currentCommand.getName();
       inputs.PreviousCommand = previousCommand.getName();
       inputs.CurrentDriveCommand = currentDriveCommand.getName();
       inputs.PreviousDriveCommand = previousDriveCommand.getName();
    }

    public static void setCommand(Command command) {
        Command temp = currentCommand;
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        currentCommand = command;
        currentCommand.schedule();
        previousCommand = temp;
    }

    public static void setDriveCommand(Command command) {
        Command temp = currentDriveCommand;
        if (currentDriveCommand != null) {
            currentCommand.cancel();
        }
        currentDriveCommand = command;
        currentDriveCommand.schedule();
        previousCommand = temp;
    }

    public static void resetRobot() {
        setCommand(Commands.none());
    }
}
