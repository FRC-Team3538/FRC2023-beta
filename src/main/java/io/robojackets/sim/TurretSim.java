package io.robojackets.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class TurretSim extends ElevatorSim {
  public TurretSim(
      LinearSystem<N2, N1, N1> plant,
      DCMotor gearbox,
      double gearRatio,
      Matrix<N1, N1> measurementStdDevs) {
    super(
        plant,
        gearbox,
        gearRatio,
        1,
        Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        false,
        measurementStdDevs);
  }

  public double getAngleRadians() {
    return getPositionMeters();
  }

  public double getAngularVelocityRadiansPerSecond() {
    return getVelocityMetersPerSecond();
  }
}
