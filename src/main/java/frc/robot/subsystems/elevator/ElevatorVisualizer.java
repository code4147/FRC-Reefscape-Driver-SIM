// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// How to use the visualizer with AdvantageScope:
// https://docs.advantagescope.org/tab-reference/mechanism

/** Class to handle graphical visualization of an elevator mechanism. */
public class ElevatorVisualizer {
    // Standard path that the visual data should be published to
    private final String LOG_KEY = "Elevator/Visualier";

    private final double kStageOneMinimumLengthMeters = 1.0470896;
    private final double kCarriageMinimumLengthMeters = 0.2469896;
    private final double kShooterMinimumLengthMeters = 0.3469896;

    /** The field that all mechanism stages are appended to. Units are in meters */
    private LoggedMechanism2d elevatorVisualField = new LoggedMechanism2d(1.0, 4.0);

    private LoggedMechanismRoot2d elevatorRoot = elevatorVisualField.getRoot("elevator", 0.5, 0.0);
    private LoggedMechanismRoot2d carrigeRoot = elevatorVisualField.getRoot("carriage", 0.6, 0);
    private LoggedMechanismLigament2d elevatorFirstStage = elevatorRoot.append(new LoggedMechanismLigament2d(
            "elevantorstage", kStageOneMinimumLengthMeters, 90.0, 4, new Color8Bit(Color.kWhite)));
    private LoggedMechanismLigament2d elevatorCarriage = carrigeRoot.append(new LoggedMechanismLigament2d(
            "Carriege", kCarriageMinimumLengthMeters, 90.0, 4, new Color8Bit(Color.kBlue)));
    private LoggedMechanismLigament2d shooter = elevatorCarriage.append(
            new LoggedMechanismLigament2d("shooter", kShooterMinimumLengthMeters, 0.0, 4, new Color8Bit(Color.kBlack)));
    private LoggedMechanismLigament2d asdf = shooter.append(new LoggedMechanismLigament2d(
            "shooter", kShooterMinimumLengthMeters, -120.0, 4, new Color8Bit(Color.kBlack)));

    /**
     * Creates a new visualizer
     *
     * @param initialPosition The starting position of the arm
     */
    public ElevatorVisualizer(double initialPositionMeters) {
        // elevatorCarriage.setLength(kStageOneMinimumLengthMeters + initialPositionMeters);

        Logger.recordOutput(LOG_KEY, elevatorVisualField);
    }

    /**
     * Updates the position of the elevator on the visualizer
     *
     * @param positionMeters The current position of the elevator mechanism
     */
    public void updateElevatorPosition(double positionMeters) {
        if (kStageOneMinimumLengthMeters < positionMeters + kCarriageMinimumLengthMeters) {

            elevatorFirstStage.setLength(kStageOneMinimumLengthMeters
                    + (positionMeters - kStageOneMinimumLengthMeters + kCarriageMinimumLengthMeters)
                    + 0.0508);
        }

        elevatorCarriage.setLength(kCarriageMinimumLengthMeters + positionMeters);

        Logger.recordOutput(LOG_KEY, elevatorVisualField);
    }

    // /** @return The current length that the first stage mechanism visual is set to */
    // @AutoLogOutput(key = LOG_KEY + "/FirstStageLength")
    // public double getFirstStageLength() {
    //     return elevatorFirstStage.getLength();
    // }

    // /** @return The current length that the second stage mechanism visual is set to */
    // @AutoLogOutput(key = LOG_KEY + "/SecondStageLength")
    // public double getSecondStageLength() {
    //     return elevatorCarriage.getLength();
    // }
}
