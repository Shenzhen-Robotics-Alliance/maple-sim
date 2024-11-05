package org.ironmaple.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 *
 *
 * <h1>{@link DCMotorSim} with a bit of extra spice.</h1>
 *
 * <p>This class extends the functionality of the original {@link DCMotorSim} and models the
 * following aspects in addition:
 *
 * <ul>
 *   <li>Friction force on the rotor.
 *   <li>Smart current limiting.
 *   <li>Brake and coast modes (only for simulating brushless motors).
 * </ul>
 */
public class MapleMotorSim {
  public static enum OutputType {
    VOLTAGE,
    CURRENT
  }

  public static enum OutputMode {
    VELOCITY,
    POSITION,
    OPEN_LOOP
  }

  /** The Constants for the motor */
  private final DCMotor motor;
  /** The dynamics simulation for the motor */
  private final DCMotorSim sim;
  /** The gear ratio, value above 1.0 are a reduction */
  private final double gearing;
  /** The voltage required to overcome friction */
  private final Voltage frictionVoltage;

  private final PIDController poseVoltController = new PIDController(0, 0, 0);
  private final PIDController veloVoltController = new PIDController(0, 0, 0);
  private final PIDController poseCurrentController = new PIDController(0, 0, 0);
  private final PIDController veloCurrentController = new PIDController(0, 0, 0);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  private Current currentLimit = Amps.of(300.0);

  private OutputType outputType = OutputType.VOLTAGE;
  private OutputMode outputMode = OutputMode.OPEN_LOOP;
  private double output = 0.0;

  private Angle forwardLimit = Radians.of(Double.POSITIVE_INFINITY);
  private Angle reverseLimit = Radians.of(Double.NEGATIVE_INFINITY);

  /**
   *
   *
   * <h2>Constructs a Brushless Motor Simulation Instance.</h2>
   *
   * @param motor the {@link DCMotor} model representing the motor(s) in the simulation
   * @param gearRatio the gear ratio of the mechanism; values greater than 1 indicate a reduction
   * @param loadIntertia the rotational inertia of the mechanism
   * @param frictionVoltage the voltage required to keep the motor moving at a constant velocity
   */
  public MapleMotorSim(
      SimulatedArena arena,
      DCMotor motor,
      double gearRatio,
      MomentOfInertia loadIntertia,
      Voltage frictionVoltage) {
    this.sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor, loadIntertia.in(KilogramSquareMeters), gearRatio),
            motor);
    this.motor = motor;
    this.gearing = gearRatio;
    this.frictionVoltage = frictionVoltage;

    arena.addMotor(this);
  }

  public MapleMotorSim withFeedForward(
      Voltage kS,
      Per<VoltageUnit, AngularVelocityUnit> kV,
      Per<VoltageUnit, AngularAccelerationUnit> kA) {
    var kVUnit = PerUnit.combine(Volts, RadiansPerSecond);
    var kAUnit = PerUnit.combine(Volts, RadiansPerSecondPerSecond);
    feedforward =
        new SimpleMotorFeedforward(
            kS.in(Volts), kV.in(kVUnit), kA.in(kAUnit), SimulatedArena.getSimulationDt());
    return this;
  }

  /**
   * Configures the PD controller for Positional Requests using {@link OutputType#VOLTAGE}.
   *
   * <p>This is unit safe and can be configure like so:
   *
   * <pre><code>
   * // Volts per Rotation of error is how CTRE handles PID when used with voltage requests
   * sim.withPositionalVoltageController(
   *   Volts.per(Rotation).ofNative(100.0),
   *   Volts.per(RotationsPerSecond).ofNative(5.0)
   * );
   * </code></pre>
   *
   * @param kP the proportional gain
   * @param kD the derivative gain
   * @return this instance for method chaining
   */
  public MapleMotorSim withPositionalVoltageController(
      Per<VoltageUnit, AngleUnit> kP, Per<VoltageUnit, AngularVelocityUnit> kD) {
    var kPUnit = PerUnit.combine(Volts, Radians);
    var kDUnit = PerUnit.combine(Volts, RadiansPerSecond);
    poseVoltController.setP(kP.in(kPUnit));
    poseVoltController.setD(kD.in(kDUnit));
    return this;
  }

  /**
   * Configures the PD controller for Velocity Requests using {@link OutputType#VOLTAGE}.
   *
   * <p>This is unit safe and can be configure like so:
   *
   * <pre><code>
   * // Volts per RPS of error is how CTRE handles PID when used with voltage requests
   * sim.withVelocityVoltageController(
   *   Volts.per(RotationsPerSecond).ofNative(0.4)
   * );
   * </code></pre>
   *
   * @param kP the proportional gain
   * @return this instance for method chaining
   */
  public MapleMotorSim withVelocityVoltageController(Per<VoltageUnit, AngleUnit> kP) {
    var kPUnit = PerUnit.combine(Volts, Radians);
    veloVoltController.setP(kP.in(kPUnit));
    return this;
  }

  /**
   * Configures the PD controller for Positional Requests using {@link OutputType#CURRENT}.
   *
   * <p>This is unit safe and can be configure like so:
   *
   * <pre><code>
   * // Amps per Rotation of error is how CTRE handles PID when used with current requests
   * sim.withPositionalCurrentController(
   *   Amps.per(Rotation).ofNative(100.0),
   *   Amps.per(RotationsPerSecond).ofNative(5.0)
   * );
   * </code></pre>
   *
   * @param kP the proportional gain
   * @param kD the derivative gain
   * @return this instance for method chaining
   */
  public MapleMotorSim withPositionalCurrentController(
      Per<CurrentUnit, AngleUnit> kP, Per<CurrentUnit, AngularVelocityUnit> kD) {
    var kPUnit = PerUnit.combine(Amps, Radians);
    var kDUnit = PerUnit.combine(Amps, RadiansPerSecond);
    poseCurrentController.setP(kP.in(kPUnit));
    poseCurrentController.setD(kD.in(kDUnit));
    return this;
  }

  /**
   * Configures the PD controller for Velocity Requests using {@link OutputType#CURRENT}.
   *
   * <p>This is unit safe and can be configure like so:
   *
   * <pre><code>
   * // Amps per RPS of error is how CTRE handles PID when used with current requests
   * sim.withVelocityCurrentController(
   *   Amps.per(RotationsPerSecond).ofNative(0.4)
   * );
   * </code></pre>
   *
   * @param kP the proportional gain
   * @return this instance for method chaining
   */
  public MapleMotorSim withVelocityCurrentController(Per<CurrentUnit, AngleUnit> kP) {
    var kPUnit = PerUnit.combine(Amps, Radians);
    veloCurrentController.setP(kP.in(kPUnit));
    return this;
  }

  /**
   * Configures the positionaly controllers to use continuous wrap.
   *
   * @param min the minimum angle
   * @param max the maximum angle
   * @return this instance for method chaining
   * @see PIDController#enableContinuousInput(double, double)
   */
  public MapleMotorSim withControllerContinousInput(Angle min, Angle max) {
    poseVoltController.enableContinuousInput(min.in(Radians), max.in(Radians));
    poseCurrentController.enableContinuousInput(min.in(Radians), max.in(Radians));
    return this;
  }

  /**
   * Configures the angle of the motor.
   *
   * @param angle the angle of the motor
   * @return this instance for method chaining
   */
  public MapleMotorSim withOverrideAngle(Angle angle) {
    sim.setAngle(angle.in(Radians));
    return this;
  }

  /**
   * Configures the angular velocity of the motor.
   *
   * @param angularVelocity the angular velocity of the motor
   * @return this instance for method chaining
   */
  public MapleMotorSim withOverrideAngularVelocity(AngularVelocity angularVelocity) {
    sim.setAngularVelocity(angularVelocity.in(RadiansPerSecond));
    return this;
  }

  /**
   * Configures the current limit for the motor.
   *
   * <p>This is the total current limit for the sim
   *
   * @param currentLimit the current limit for the motor
   * @return
   */
  public MapleMotorSim withStatorCurrentLimit(Current currentLimit) {
    // this is a limit across the sum of all motors output,
    // so it should be set to the total current limit of the mechanism
    this.currentLimit = currentLimit;
    return this;
  }

  /**
   * Configures the hard limits for the motor.
   *
   * @param forwardLimit the forward limit
   * @param reverseLimit the reverse limit
   * @return this instance for method chaining
   */
  public MapleMotorSim withHardLimits(Angle forwardLimit, Angle reverseLimit) {
    this.forwardLimit = forwardLimit;
    this.reverseLimit = reverseLimit;
    return this;
  }

  public MomentOfInertia getMOI() {
    return KilogramSquareMeters.of(sim.getJKgMetersSquared());
  }

  public Angle getPosition() {
    return Radians.of(sim.getAngularPositionRad());
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
  }

  public AngularAcceleration getAcceleration() {
    return RadiansPerSecondPerSecond.of(sim.getAngularAccelerationRadPerSecSq());
  }

  public Current getStatorCurrentDraw() {
    return Amps.of(sim.getCurrentDrawAmps());
  }

  public Current getSupplyCurrent() {
    // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10
    return getStatorCurrentDraw()
        .times(sim.getInputVoltage() / RobotController.getBatteryVoltage());
  }

  public Voltage getRotorVoltage() {
    return Volts.of(sim.getInputVoltage());
  }

  public Voltage getSupplyVoltage() {
    return Volts.of(RobotController.getBatteryVoltage());
  }

  public Torque getRotorTorque(Current current) {
    return NewtonMeters.of(motor.getTorque(current.in(Amps)));
  }

  public AngularVelocity getFreeWheelSpeed() {
    return RadiansPerSecond.of(motor.freeSpeedRadPerSec);
  }

  public void setControl(OutputType outputType, AngularVelocity velo) {
    this.outputType = outputType;
    this.outputMode = OutputMode.VELOCITY;
    this.output = velo.in(RadiansPerSecond);
  }

  public void setControl(OutputType outputType, Angle pos) {
    this.outputType = outputType;
    this.outputMode = OutputMode.POSITION;
    this.output = pos.in(Radians);
  }

  public void setControl(Current amps) {
    this.outputType = OutputType.CURRENT;
    this.outputMode = OutputMode.OPEN_LOOP;
    this.output = amps.in(Amps);
  }

  public void setControl(Voltage volts) {
    this.outputType = OutputType.VOLTAGE;
    this.outputMode = OutputMode.OPEN_LOOP;
    this.output = volts.in(Volts);
  }

  public void setControl() {
    this.outputType = OutputType.VOLTAGE;
    this.outputMode = OutputMode.OPEN_LOOP;
    this.output = 0.0;
  }

  /** Package private call */
  void update() {
    double dtSeconds = SimulatedArena.getSimulationDt();
    switch (this.outputType) {
      case VOLTAGE -> {
        switch (this.outputMode) {
          case OPEN_LOOP -> {
            driveAtVoltage(Volts.of(output));
          }
          case POSITION -> {
            Voltage voltage =
                Volts.of(poseVoltController.calculate(getPosition().in(Radians), output));
            Voltage feedforwardVoltage =
                feedforward.calculate(getVelocity(), velocityForVolts(voltage));
            driveAtVoltage(feedforwardVoltage.plus(voltage));
          }
          case VELOCITY -> {
            Voltage voltage =
                Volts.of(veloVoltController.calculate(getVelocity().in(RadiansPerSecond), output));
            Voltage feedforwardVoltage =
                feedforward.calculate(getVelocity(), RadiansPerSecond.of(output));
            driveAtVoltage(voltage.plus(feedforwardVoltage));
          }
        }
      }
      case CURRENT -> {
        switch (this.outputMode) {
          case OPEN_LOOP -> {
            sim.setInputVoltage(voltsForAmps(Amps.of(output), getVelocity()).in(Volts));
          }
          case POSITION -> {
            Current current =
                Amps.of(poseCurrentController.calculate(getPosition().in(Radians), output));
            Voltage voltage = voltsForAmps(current, getVelocity());
            Voltage feedforwardVoltage =
                feedforward.calculate(getVelocity(), velocityForVolts(voltage));
            driveAtVoltage(feedforwardVoltage.plus(voltage));
          }
          case VELOCITY -> {
            Current current =
                Amps.of(veloCurrentController.calculate(getPosition().in(Radians), output));
            Voltage feedforwardVoltage =
                feedforward.calculate(getVelocity(), RadiansPerSecond.of(output));
            Voltage voltage = voltsForAmps(current, getVelocity()).plus(feedforwardVoltage);
            driveAtVoltage(voltage);
          }
        }
      }
    }

    sim.update(dtSeconds);

    if (getPosition().lte(reverseLimit)) {
      sim.setState(reverseLimit.in(Radians), 0.0);
    } else if (getPosition().gte(forwardLimit)) {
      sim.setState(forwardLimit.in(Radians), 0.0);
    }
  }

  private void driveAtVoltage(Voltage voltage) {
    // The voltage constrained to current limits and battery voltage
    Voltage constrained = constrainOutputVoltage(voltage);
    Voltage frictionVoltage = applyFriction(constrained);

    sim.setInputVoltage(frictionVoltage.in(Volts));
  }

  private Voltage applyFriction(Voltage voltage) {
    // This function is responsible for slowing down acceleration
    // and slowing down the velocity of the motor when at lowere output.
    // This is not the same as the static friction, which is the force
    // required to get the motor moving.

    // to apply friction we convert the motors output to torque then back to voltage
    double current =
        motor.getCurrent(sim.getAngularVelocityRadPerSec() * gearing, voltage.in(Volts));
    double currentVelo = getVelocity().in(RadiansPerSecond) * gearing;
    double torque = motor.getTorque(current);
    double friction = frictionTorque().in(NewtonMeters);

    boolean movingForward = currentVelo > 0;

    if (movingForward && currentVelo > motor.getSpeed(torque, sim.getInputVoltage())) {
      // the motor is moving faster than it should based on the output voltage
      // apply the friction to slow it down
      torque -= friction;
    } else if (!movingForward && currentVelo < motor.getSpeed(torque, sim.getInputVoltage())) {
      // the motor is moving slower than it should based on the output voltage
      // apply the friction to speed it up
      torque += friction;
    }

    return Volts.of(motor.getVoltage(torque, currentVelo));
  }

  private Voltage voltsForAmps(Current current, AngularVelocity angularVelocity) {
    // find what voltage is needed to get the current
    return Volts.of(
        motor.getVoltage(current.in(Amps), angularVelocity.in(RadiansPerSecond) * gearing));
  }

  private AngularVelocity velocityForVolts(Voltage voltage) {
    return RadiansPerSecond.of(
        motor.getSpeed(motor.getTorque(getStatorCurrentDraw().in(Amps)), voltage.in(Volts)));
  }

  private Torque frictionTorque() {
    return NewtonMeters.of(
        motor.getTorque(motor.getCurrent(0.0, frictionVoltage.in(Volts))) * gearing);
  }

  private Voltage constrainOutputVoltage(Voltage requestedOutput) {
    final double kCurrentThreshold = 1.2;

    final double motorCurrentVelocityRadPerSec = getVelocity().in(RadiansPerSecond);
    final double currentLimitAmps = currentLimit.in(Amps);
    final double requestedOutputVoltage = requestedOutput.in(Volts);
    final double currentAtRequestedVolts =
        motor.getCurrent(motorCurrentVelocityRadPerSec, requestedOutputVoltage);

    // Resource for current limiting:
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
    final boolean currentTooHigh =
        Math.abs(currentAtRequestedVolts) > (kCurrentThreshold * currentLimitAmps);
    double limitedVoltage = requestedOutputVoltage;
    if (currentTooHigh) {
      final double limitedCurrent = Math.copySign(currentLimitAmps, currentAtRequestedVolts);
      limitedVoltage =
          motor.getVoltage(motor.getTorque(limitedCurrent), motorCurrentVelocityRadPerSec);
    }

    // ensure the current limit doesn't cause an increase to output voltage
    if (Math.abs(limitedVoltage) > Math.abs(requestedOutputVoltage)) {
      limitedVoltage = requestedOutputVoltage;
    }

    // constrain the output voltage to the battery voltage
    return Volts.of(
        MathUtil.clamp(
            limitedVoltage,
            -RobotController.getBatteryVoltage(),
            RobotController.getBatteryVoltage()));
  }
}
