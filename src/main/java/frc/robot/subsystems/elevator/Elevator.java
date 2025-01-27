// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Isaac Is Cool

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    public enum ElevatorGoal {
        L1(() -> 0),
        L2(() -> 0.36),
        L3(() -> 0.8);

        private DoubleSupplier goalMeters;

        ElevatorGoal(DoubleSupplier goalMeters) {
            this.goalMeters = goalMeters;
        }

        private double getGoalMeters() {
            return this.goalMeters.getAsDouble();
        }
    }
    private final ElevatorIO kHardware;
    private final ElevatorIOInputsAutoLogged kInputs = new ElevatorIOInputsAutoLogged();
    private ElevatorGoal currentElevaotrGoal = null;
    private double currentElevatorGoalPositionMeters = 0.0;
    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("Elevator/Gains/kP", ElevatorConstants.kElevatorGains.kP());
    private final LoggedTunableNumber kI =
            new LoggedTunableNumber("Elevator/Gains/kI", ElevatorConstants.kElevatorGains.kI());
    private final LoggedTunableNumber kD =
            new LoggedTunableNumber("Elevator/Gains/kD", ElevatorConstants.kElevatorGains.kD());
    private final LoggedTunableNumber kS =
            new LoggedTunableNumber("Elevator/Gains/kS", ElevatorConstants.kElevatorGains.kS());
    private final LoggedTunableNumber kV =
            new LoggedTunableNumber("Elevator/Gains/kV", ElevatorConstants.kElevatorGains.kV());
    private final LoggedTunableNumber kA =
            new LoggedTunableNumber("Elevator/Gains/kA", ElevatorConstants.kElevatorGains.kA());
    private final LoggedTunableNumber kG =
            new LoggedTunableNumber("Elevator/Gains/kG", ElevatorConstants.kElevatorGains.kG());
    private final LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber(
            "Elevator/MotionMagic/kMaxVelocity", ElevatorConstants.kElevatorGains.kMaxVelocityMetersPerSecond());
    private final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber(
            "Elevator/MotionMagic/kMaxAcceleration",
            ElevatorConstants.kElevatorGains.kMaxAccelerationMetersPerSecondSquared());
    private final ElevatorVisualizer kVisualizer;
    public Elevator(ElevatorIO io) {
        kHardware = io;
        kVisualizer = new ElevatorVisualizer(getPositionMeters());
    }
    @Override
    public void periodic() {
        kHardware.updateInputs(kInputs);
        Logger.processInputs("Elevator/Inputs", kInputs);

        if (currentElevaotrGoal != null) {
            currentElevatorGoalPositionMeters = currentElevaotrGoal.getGoalMeters();
            setPosition(currentElevatorGoalPositionMeters);

            Logger.recordOutput("Elevator/Goal", currentElevaotrGoal);
        } else {
            Logger.recordOutput("Elevator/Goal", "NONE");
        }
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    kHardware.setGains(kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get());
                },kP,kI,kD,kS,kV,kA,kG);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    kHardware.setMotionMagicConstraints(kMaxVelocity.get(), kMaxAcceleration.get());
                },
                kMaxVelocity,
                kMaxAcceleration);
        kVisualizer.updateElevatorPosition(getPositionMeters());
    }
    public void setGoal(ElevatorGoal desiredGoal) {
        currentElevaotrGoal = desiredGoal;
    }
    public void setVoltage(double voltage) {
        if (getPositionMeters() > ElevatorConstants.kMaxPositionMeters && voltage > 0.0) {
            return;
        } else if (getPositionMeters() < ElevatorConstants.kMinPositionMeters && voltage < 0.0) {
            return;
        } else {
            kHardware.setVoltage(voltage);
        }
    }
    public void stop() {
        currentElevaotrGoal = null;
        kHardware.stop();
    }
    private void setPosition(double positionGoalMeters) {
        kHardware.setPosition(positionGoalMeters);
    }
    public double getPositionMeters() {
        return kInputs.positionMeters;
    }
    @AutoLogOutput(key = "Elevator/Feedback/GoalMeters")
    public double getPositionGoalMeters() {
        return currentElevatorGoalPositionMeters;
    }
}
