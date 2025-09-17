// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SetModeCommand;
import frc.robot.commands.driveCommands.DriveCommandsFunctions;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.DrivetrainPublisher;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import javax.swing.text.html.HTMLDocument;
import java.io.IOException;
import java.util.Map;

import static java.util.Map.entry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> sysIDChooser;


    private Command rightL1;
    private Command leftL1;
    private Command rightL1NoExtras;

    public final static String rightL1Name = "Right L1";
    public final static String leftL1Name = "Left L1";
    public final static String rightL1NoExtrasName = "Right L1 No Extras";

    public PathPlannerPath testPath;

    private final LoggedDashboardChooser<Command> autonSelector = new LoggedDashboardChooser<>("Auto Chooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set up auto routines
    sysIDChooser = new LoggedDashboardChooser<>("SysID Choices");

    // Set up SysId routines
    sysIDChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommandsFunctions.wheelRadiusCharacterization(Robot.drivetrain));
    sysIDChooser.addOption(
        "Drive Simple FF Characterization", DriveCommandsFunctions.feedforwardCharacterization(Robot.drivetrain));
    sysIDChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        Robot.drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    sysIDChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        Robot.drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    sysIDChooser.addOption(
        "Drive SysId (Dynamic Forward)", Robot.drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    sysIDChooser.addOption(
        "Drive SysId (Dynamic Reverse)", Robot.drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      setUpPathplannerCommands();

//        autonSelector.addOption("Left Triple L4", Autos.J4_K4_L4_CoralStation(autoFactory));
//        autonSelector.addOption("Right Triple L4", Autos.E4_C4_D4_CoralStation(autoFactory));
      autonSelector.addOption("Center G4", Autos.centerAutoG());
      autonSelector.addOption("Center H4", Autos.centerAutoH());
//        autonSelector.addOption("Lolipops left", Autos.J4_L4_A4_B4_Lolipops());
      autonSelector.addOption("Lolipops left Pathplanner Routine", AutoBuilder.buildAuto("Left Lolipops"));
      autonSelector.addOption("Lolipops left Pathplanner Routine 3L4",
              AutoBuilder.buildAuto("Left Lolipops With 3L4"));
      autonSelector.addOption("Lolipops Right 3L4",  AutoBuilder.buildAuto("Right Lolipops"));
      autonSelector.addOption("Steal Coral Auto", AutoBuilder.buildAuto("Steal Coral Auton"));

      setupTestPath();

      // Robot.robotMode.setCurrentMode(RobotMode.initialTransitPose);
      // Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);

      Robot.limelight.initialPoseEstimates();

    // Configure the button bindings
    configureButtonBindings();
  }


    private void setUpPathplannerCommands() {

        Command TransitPose = new SetModeCommand(RobotControl.transitPose);
        Command WaitForCoral = new WaitUntilCommand(TriggerBoard::isCoralInManipulator);
        Command WaitForCoralFloorPose = Commands.print("waiting for coral floor pose").repeatedly()
                .withDeadline(new WaitUntilCommand(() -> RobotControl.coralFloorPose.isScheduled()));
        Command AlignWithA4 = new SetModeCommand(Autos.alignWithA4);
        Command AlignWithB4 = new SetModeCommand(Autos.alignWithB4);
        Command AlignWithA2 = new SetModeCommand(Autos.alignWithA2);
        Command AlignWithB2 = new SetModeCommand(Autos.alignWithB2);
        Command AlignWithL4 = new SetModeCommand(Autos.alignWithL4);
        Command AlignWithK4 = new SetModeCommand(Autos.alignWithK4);
        Command AlignWithD4 = new SetModeCommand(Autos.alignWithD4);
        Command AlignWithF4 = new SetModeCommand(Autos.alignWithF4);
        Command AlignWithE4 = new SetModeCommand(Autos.alignWithE4);
        Command AlignWithC4 = new SetModeCommand(Autos.alignWithC4);

        Map<String, Command> pathPlannerCommands = Map.ofEntries(
                entry("TransitPose", TransitPose),
                entry("AlignWithA4", AlignWithA4),
                entry("AlignWithB4", AlignWithB4),
                entry("AlignWithA2", AlignWithA2),
                entry("AlignWithB2", AlignWithB2),
                entry("AlignWithL4", AlignWithL4),
                entry("AlignWithK4", AlignWithK4),
                entry("AlignWithD4", AlignWithD4),
                entry("AlignWithE4", AlignWithE4),
                entry("AlignWithF4", AlignWithF4),
                entry("AlignWithC4", AlignWithC4),
                entry("WaitForCoral", WaitForCoral),
                entry("WaitForCoralFloorPose", WaitForCoralFloorPose));

        NamedCommands.registerCommands(pathPlannerCommands);
    }

    private void setupTestPath() {
        try {
            testPath = PathPlannerPath.fromPathFile("Test Path");
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      Robot.joystick.start().onTrue(new InstantCommand(() -> RobotControl.setCurrentMode(RobotControl.resetPose)));
      Robot.joystick.back().onTrue(Robot.drivetrain.runOnce(() -> {
          Robot.drivetrain.setPose(new Pose2d());
          Robot.limelight.resetIMU();
      }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return autonSelector.get();
  }
}
