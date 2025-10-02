package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {

    private final DCMotor elevatorGearbox = DCMotor.getKrakenX60(2);

    private final DCMotorSim elevatorSim;

    private final PIDController elevatorController = new PIDController(50, 0.0, 10);

    private double elevatorAppliedVolts = 0.0;
    private boolean elevatorClosedLoop = false;

    public ElevatorIOSim() {
        elevatorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(elevatorGearbox, 1, 1), elevatorGearbox);
        elevatorController.setTolerance(0.005);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (elevatorClosedLoop) {
            elevatorAppliedVolts = elevatorController.calculate(elevatorSim.getAngularPositionRotations());
        } else {
            elevatorController.reset();
        }

        elevatorSim.setInputVoltage(MathUtil.clamp(elevatorAppliedVolts, -12.0, 12.0));
        elevatorSim.update(0.02);

        inputs.elevatorPositionMeters = elevatorSim.getAngularPositionRotations();
        inputs.elevatorVelocityMetersPerSec = elevatorSim.getAngularVelocityRadPerSec();
        inputs.elevatorAppliedVolts = elevatorAppliedVolts;
        inputs.elevatorSetHeightMeters = elevatorController.getSetpoint();
        inputs.elevatorError = elevatorController.getError();
        inputs.elevatorAtSetpoint = elevatorController.atSetpoint();
        inputs.elevatorCurrent = elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public void setHeight(double meters) {
        elevatorClosedLoop = true;
        elevatorController.setSetpoint(meters);
    }

    @Override
    public void setSpeed(double speed) {
        elevatorClosedLoop = false;
        elevatorAppliedVolts = speed * 12;
    }
}
