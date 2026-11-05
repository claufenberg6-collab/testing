package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Flywheel subsystem using SparkMAX with NEO motor
 */
@Logged(name = "flywheel1")
public class flywheel1 extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getNEO(1);
  private final int canID = 11;
  private final double gearRatio = 2;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
  private final double kS = 0;
  private final double kV = 0;
  private final double kA = 0;
  private final double kG = 0; // Unused for flywheels
  private final double maxVelocity = 20; // rad/s
  private final double maxAcceleration = 20; // rad/s²
  private final boolean brakeMode = false;
  private final boolean enableStatorLimit = true;
  private final double statorCurrentLimit = 40;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40;
  private final double momentOfInertia = 32; // kg*m^2, flywheel moment of inertia


  // Feedforward
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    kS,
    kV,
    kA
  );

  // Motor controller
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkSim motorSim;
  private final SparkClosedLoopController sparkPidController;

  // Simulation
  private final FlywheelSim flywheelSim;

  /**
   * Creates a new Flywheel Subsystem.
   */
  public flywheel1() {
    // Initialize motor controller
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motor = new SparkMax(canID, MotorType.kBrushless);
    motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set current limits
    motorConfig.secondaryCurrentLimit( statorCurrentLimit);

    // Configure Feedback and Feedforward
    sparkPidController = motor.getClosedLoopController();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
    motorConfig.closedLoop.feedForward.kG(kG);

    // Configure Encoder Gear Ratio
    motorConfig.encoder
      .positionConversionFactor(1 / gearRatio)
      .velocityConversionFactor((1 / gearRatio) / 60); // Covnert RPM to RPS

    // Save configuration
    motor.configure(
      motorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    motorSim = new SparkSim(motor, dcMotor);

    // Initialize simulation
   flywheelSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(dcMotor, momentOfInertia, gearRatio),
    dcMotor
);

}
   
  

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {}

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Use getVoltage() for other controllers
    flywheelSim.setInput(getVoltage());

    // Update simulation by 20ms
    flywheelSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        flywheelSim.getCurrentDrawAmps()
      )
    );

    double motorVelocity = RadiansPerSecond.of(
      flywheelSim.getAngularVelocityRadPerSec() * gearRatio
    ).in(RotationsPerSecond);
    double motorPosition = 0; // Flywheels do not track absolute position
    motorSim.iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Get the current velocity in RPM.
   * @return Velocity in RPM
   */
  @Logged(name = "Velocity/RPM")
  public double getVelocityRPM() {
    return getVelocity() * 60.0;
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  /**
   * Set flywheel velocity.
   * @param velocityRPM The target velocity in RPM
   */
  public void setVelocity(double velocityRPM) {
    // Convert RPM to rotations per second
    double velocityRotations = velocityRPM / 60.0;

    double acceleration = 0;
    sparkPidController.setSetpoint(
      velocityRotations,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the flywheel simulation for testing.
   * @return The flywheel simulation model
   */
  public FlywheelSim getSimulation() {
    return flywheelSim;
  }

  /**
   * Checks whether the flywheel is within tolerance of a target velocity.
   * @param velocityRPM The target velocity in RPM
   * @return True if within 2% (or 50 RPM, whichever is greater) of the target velocity
   */
  public boolean atTargetVelocity(double velocityRPM) {
    return (
      Math.abs(getVelocityRPM() - velocityRPM) <
      Math.max(50.0, Math.abs(velocityRPM) * 0.02)
    );
  }

  /**
   * Creates a command to spin the flywheel up to a specific velocity.
   * @param velocityRPM The target velocity in RPM
   * @return A command that spins the flywheel to the specified velocity
   */
  public Command spinUpCommandf1(double velocityRPM) {
    return run(() -> setVelocity(velocityRPM)).until(() ->
      atTargetVelocity(velocityRPM)
    );
  }

  /**
   * Creates a command to stop the flywheel.
   * @return A command that stops the flywheel
   */
  public Command stopCommandf1() {
    return runOnce(() -> setVelocity(0));
  }
}
