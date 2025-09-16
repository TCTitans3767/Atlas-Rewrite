package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ClimberIOHardware implements ClimberIO{

    private TalonFX rightMotor, leftMotor;
    private final TalonFXConfiguration rightConfig, leftConfig;
    // Private final TalonFXConfiguration rightConfig, leftConfig;
    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    private double targetRotations = 0;

    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<AngularVelocity> climberVelocity;
    private final StatusSignal<Voltage> climberAppliedVolts;
    private final StatusSignal<Current> climberCurrent;

    public ClimberIOHardware(){
        // Initialize the motors
        leftMotor = new TalonFX(Constants.Climber.leftMotorID);
        leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Climber.conversonFactor;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        slot0Config = new Slot0Configs();
        slot0Config.kP = 80;
        slot0Config.kI = 0;
        slot0Config.kD = 0;
        slot0Config.GravityType = GravityTypeValue.Elevator_Static;
        slot0Config.kG = 0; // Gravity
        slot0Config.kS = 0;


        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Climber.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Climber.maxAcceleration;


        leftMotor.getConfigurator().apply(leftConfig);
        leftMotor.getConfigurator().apply(slot0Config);
        leftMotor.getConfigurator().apply(motionMagicConfig);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor = new TalonFX(Constants.Climber.rightMotorID);
        rightMotor.getConfigurator().apply(rightConfig);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setControl(new Follower(Constants.Climber.leftMotorID, true));

        climberPosition = leftMotor.getPosition();
        climberVelocity = leftMotor.getVelocity();
        climberAppliedVolts = leftMotor.getMotorVoltage();
        climberCurrent = leftMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Climber.ODOMETRY_FREQUENCY, climberCurrent, climberPosition, climberVelocity, climberAppliedVolts);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberRotations = climberPosition.getValueAsDouble();
        inputs.climberVelocityRotationsPerSec = climberVelocity.getValueAsDouble();
        inputs.climberCurrent = climberCurrent.getValueAsDouble();
        inputs.climberAppliedVolts = climberAppliedVolts.getValueAsDouble();
        inputs.climberSetpoint = targetRotations;
        inputs.climberAtSetpoint = MathUtil.isNear(targetRotations, inputs.climberRotations, Constants.Climber.error_tolerance);
    }

    @Override
    public void setClimberPosition(double position) {
        leftMotor.setControl(new MotionMagicVoltage(position));
    }

    @Override
    public void setClimberSpeed(double speed) {
        leftMotor.set(speed);
    }
}
