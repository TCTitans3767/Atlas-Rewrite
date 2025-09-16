package frc.robot.subsystems.robotControl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.driveCommands.*;
import frc.robot.commands.modes.*;
import frc.robot.commands.transitions.*;
import org.littletonrobotics.junction.Logger;

public class RobotControl extends SubsystemBase implements RobotControlIO{

    private final RobotControlIOInputsAutoLogged inputs = new RobotControlIOInputsAutoLogged();
    private final RobotControlIO io = this;

    private static Command currentCommand = Commands.none();
    private static Command currentDriveCommand = Commands.none();
    private static Command previousCommand = Commands.none();
    private static Command previousDriveCommand = Commands.none();

    public static AlignWithLeftReef alignWithLeftReef;
    public static AlignWithRightReef alignWithRightReef;
    public static ControllerDrive controllerDrive;

    public static CoralStation coralStation;
    public static TransitPose transitPose;
    public static Transit transit;
    public static CoralStationPose coralStationPose;
    public static CoralFloorPose coralFloorPose;
    public static CoralFloor coralFloor;
    public static CoralReefPose coralReefPose;
    public static CoralReef coralReef;
    public static CoralReefAlignPose coralReefAlignPose;
    public static CoralReefAligned coralReefAligned;
    public static ScoreCoralPose scoreCoralPose;
    public static AlgaePickupPose algaePickupPose;
    public static AlgaePickup algaePickup;
    public static InitialTransitPose initialTransitPose;
    public static CoralRecievedPose coralRecievedPose;
    public static AlignWithAlgae alignWithAlgae;
    public static Climb climb;
    public static FinalClimb finalClimb;
    public static DeployClimberPose deployClimberPose;
    public static ClimbPose climbPose;
    public static EjectAlgaePose ejectAlgaePose;

    public static KnockOffAlgaePose knockOffAlgaePose;
    public static KnockOffAlgaePoseManual knockOffAlgaePoseManual;

    public static SlowControllerDrive slowControllerDrive;

    public static L1Pose L1Pose;
    public static L1 L1;

    public static ResetPose resetPose;

    public RobotControl() {

        alignWithLeftReef = new AlignWithLeftReef();
        alignWithRightReef = new AlignWithRightReef();
        controllerDrive = new ControllerDrive();

        coralStation = new CoralStation();
        transitPose = new TransitPose();
        transit = new Transit();
        coralStationPose = new CoralStationPose();
        coralFloorPose = new CoralFloorPose();
        coralFloor = new CoralFloor();
        coralReefPose = new CoralReefPose();
        coralReef = new CoralReef();
        coralReefAlignPose = new CoralReefAlignPose();
        coralReefAligned = new CoralReefAligned();
        scoreCoralPose = new ScoreCoralPose();
        algaePickupPose = new AlgaePickupPose();
        algaePickup = new AlgaePickup();
        initialTransitPose = new InitialTransitPose();
        coralRecievedPose = new CoralRecievedPose();
        alignWithAlgae = new AlignWithAlgae();
        knockOffAlgaePose = new KnockOffAlgaePose();
        climb = new Climb();
        finalClimb = new FinalClimb();
        deployClimberPose = new DeployClimberPose();
        climbPose = new ClimbPose();
        ejectAlgaePose = new EjectAlgaePose();

        knockOffAlgaePoseManual = new KnockOffAlgaePoseManual();

        slowControllerDrive = new SlowControllerDrive();

        L1Pose = new L1Pose();
        L1 = new L1();

        resetPose = new ResetPose();
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

    public static void setCurrentMode(Command command) {
        Command temp = currentCommand;
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        currentCommand = command;
        currentCommand.schedule();
        previousCommand = temp;
    }

    public static void setDriveModeCommand(Command command) {
        Command temp = currentDriveCommand;
        if (currentDriveCommand != null) {
            currentCommand.cancel();
        }
        currentDriveCommand = command;
        currentDriveCommand.schedule();
        previousCommand = temp;
    }

    public static void resetRobot() {
        setCurrentMode(Commands.none());
    }

    public static boolean isDriveCommandFinished() {
        return currentDriveCommand.isFinished();
    }
}
