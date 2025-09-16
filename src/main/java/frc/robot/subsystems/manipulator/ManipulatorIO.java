package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {

    @AutoLog
    public static class ManipulatorIOInputs {
        public double manipulatorWheelSpeed = 0.0;
        public double manipulatorAppliedVolts = 0.0;
        public double manipulatorCurrent = 0.0;
        public boolean manipulatorHasCoral = false;
        public boolean manipulatorHasAlgae = false;
        public double manipulatorSetSpeed = 0.0;
    }
    public default void updateInputs(ManipulatorIOInputs inputs) {}

    public default void setManipulatorWheelSpeed (double speed) {}
}
