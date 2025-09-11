package frc.robot.subsystems.robotControl;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface RobotControlIO {
    @AutoLog
    public static class RobotControlIOInputs {
        public String CurrentCommand = "";
        public String PreviousCommand = "";
        public String CurrentDriveCommand = "";
        public String PreviousDriveCommand = "";
    }

    public void updateInputs(RobotControlIOInputs inputs);

}
