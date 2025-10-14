package frc.robot.subsystems.robotControl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.driveCommands.*;
import frc.robot.commands.modes.*;
import frc.robot.commands.transitions.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotControl extends SubsystemBase implements RobotControlIO{

    private final RobotControlIOInputsAutoLogged inputs = new RobotControlIOInputsAutoLogged();
    private final RobotControlIO io = this;

    public static Command currentCommand = Commands.none();
    public static Command currentDriveCommand = Commands.none();
    private static Command previousCommand = Commands.none();
    private static Command previousDriveCommand = Commands.none();

    public static AlignWithLeftReef alignWithLeftReef;
    public static AlignWithRightReef alignWithRightReef;
    public static ControllerDrive controllerDrive;
    public static SlowControllerDrive slowControllerDrive;
    public static AlignWithAlgae alignWithAlgae;

    public RobotControl() {

        alignWithLeftReef = new AlignWithLeftReef();
        alignWithRightReef = new AlignWithRightReef();
        controllerDrive = new ControllerDrive();
        slowControllerDrive = new SlowControllerDrive();
        alignWithAlgae = new AlignWithAlgae();
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
       inputs.currentCommandRunning = currentCommand.isScheduled();
    }

    public static void setCurrentMode(Command command) {
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        previousCommand = currentCommand;
        currentCommand = command;
        currentCommand.schedule();
    }

    public static void setDriveModeCommand(Command command) {
        if (currentDriveCommand != null) {
            currentDriveCommand.cancel();
        }
        previousDriveCommand = currentDriveCommand;
        currentDriveCommand = command;
        currentDriveCommand.schedule();
    }

    public static void resetRobot() {
        setCurrentMode(Commands.none());
    }

    public static boolean isDriveCommandFinished() {
        return currentDriveCommand.isFinished();
    }

}
