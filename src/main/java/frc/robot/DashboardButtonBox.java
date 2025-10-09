package frc.robot;

import java.io.IOException;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Utils.ReefPosition;
import org.json.simple.parser.ParseException;

public class DashboardButtonBox{

    private static ReefPosition selectedReef = ReefPosition.A;
    private static int selectedLevel = 0;
    private static int lastSelectedLevel = 0;
    private static DashboardController dashboardController;

    public DashboardButtonBox(int controllerPort, String name) {
        dashboardController = new DashboardController(controllerPort, name);
        try {
            dashboardController.buildFromJSON();
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static ReefPosition getSelectedReefBranch() {
        for (int i = dashboardController.getButtonID("A"); i <= dashboardController.getButtonID("L"); i++) {
            if (dashboardController.isButtonPressed(i)) selectedReef = Constants.ButtonBoxButtons.branchButtonMap.get(i);
        }
        return selectedReef;
    }

    public static int getSelectedReefLevel() {
        lastSelectedLevel = selectedLevel;
        for (int i = dashboardController.getButtonID("L1"); i <= dashboardController.getButtonID("L4"); i++) {
            if (dashboardController.isButtonPressed(i)) selectedLevel = i;
        }
        return selectedLevel;
    }

    public static String getSelectedLevelString() {
        return String.valueOf(selectedLevel);
    }

    public static boolean isAlgaeKnockoffOn() {
        if (DriverStation.isAutonomousEnabled()) {
            return false;
        }
        return dashboardController.isButtonPressed(Constants.ButtonBoxButtons.algaeEject);
    }

    public static boolean hasSelectedLevelChanged() {
        return lastSelectedLevel != selectedLevel;
    }

    public static boolean isClimbPressed() {
        return dashboardController.isButtonPressed(Constants.ButtonBoxButtons.climb);
    }

    public static boolean getRawButton(int buttonID) {
        return dashboardController.isButtonPressed(buttonID);
    }

    public void buttonBoxPeriodic() {
        dashboardController.update();
    }
}
