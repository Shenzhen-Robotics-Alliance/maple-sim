package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.ControlRequest;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 *
 *
 * <h2>Simulation for a Single Swerve Module.</h2>
 *
 * <p>This class provides a simulation for a single swerve module in the {@link SwerveDriveSimulation}.
 *
 * <h3>1. Purpose</h3>
 *
 * <p>This class serves as the bridge between your code and the physics engine.
 *
 * <p>You will apply voltage outputs to the drive/steer motor of the module and obtain their encoder readings in your
 * code, just as how you deal with your physical motors.
 *
 * <h3>2. Perspectives</h3>
 *
 * <ul>
 *   <li>Simulates the steering mechanism using a custom brushless motor simulator.
 *   <li>Simulates the propelling force generated by the driving motor, with a current limit.
 *   <li>Simulates encoder readings, which can be used to simulate a {@link SwerveDriveOdometry}.
 * </ul>
 *
 * <h3>3. Simulating Odometry</h3>
 *
 * <ul>
 *   <li>Retrieve the encoder readings from {@link #getDriveEncoderUnGearedPosition()}} and
 *       {@link #getSteerAbsoluteFacing()}.
 *   <li>Use {@link SwerveDriveOdometry} to estimate the pose of your robot.
 *   <li><a
 *       href="https://v6.docs.ctr-electronics.com/en/latest/docs/application-notes/update-frequency-impact.html">250Hz
 *       Odometry</a> is supported. You can retrive cached encoder readings from every sub-tick through
 *       {@link #getCachedDriveEncoderUnGearedPositions()} and {@link #getCachedSteerAbsolutePositions()}.
 * </ul>
 *
 * <p>An example of how to simulate odometry using this class is the <a
 * href='https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java'>ModuleIOSim.java</a>
 * from the <code>Advanced Swerve Drive with maple-sim</code> example.
 */
public class SwerveModuleSimulation {
    public final SimMotorConfigs driveMotorConfigs;
    public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEELS_COEFFICIENT_OF_FRICTION;
    public final Current DRIVE_CURRENT_LIMIT;
    public final Voltage DRIVE_FRICTION_VOLTAGE;
    public final Distance WHEEL_RADIUS;

    private final MapleMotorSim steerMotorSim;

    private Voltage driveMotorAppliedVoltage = Volts.zero();
    private Current driveMotorStatorCurrent = Amps.zero();
    private Angle driveWheelFinalPosition = Radians.zero();
    private AngularVelocity steerVelocity = RadiansPerSecond.zero(), driveWheelFinalSpeed = RadiansPerSecond.zero();

    private ControlRequest driveMotorRequest = new ControlRequest.VoltageOut(Volts.zero());

    private Rotation2d steerAbsoluteFacing = Rotation2d.fromRotations(Math.random());
    private final Angle steerRelativeEncoderOffSet = Radians.of((Math.random() - 0.5) * 30);
    private final Queue<Angle> driveWheelFinalPositionCache;
    private final Queue<Rotation2d> steerAbsolutePositionCache;

    /**
     *
     *
     * <h2>Constructs a Swerve Module Simulation.</h2>
     *
     * <p>If you are using {@link SimulatedArena#overrideSimulationTimings(Time, int)} to use custom timings, you must
     * call the method before constructing any swerve module simulations using this constructor.
     *
     * @param driveMotor the model of the driving motor
     * @param steerMotor the model of the steering motor
     * @param driveGearRatio the gear ratio for the driving motor, >1 is reduction
     * @param steerGearRatio the gear ratio for the steering motor, >1 is reduction
     * @param driveCurrentLimit the stator current limit for the driving motor
     * @param steerCurrentLimit the stator current limit for the steering motor
     * @param driveFrictionVoltage the measured minimum amount of voltage that can turn the driving rotter
     * @param steerFrictionVoltage the measured minimum amount of voltage that can turn the steering rotter
     * @param wheelRadius the radius of the wheels.
     * @param steerRotationalInertia the rotational inertia of the entire steering mechanism
     * @param tireCoefficientOfFriction the <a
     *     href='https://simple.wikipedia.org/wiki/Coefficient_of_friction#:~:text=A%20coefficient%20of%20friction%20is%20a%20value%20that%20shows%20the'>coefficient
     *     of friction</a> of the tires, normally around 1.2 {@link Units#inchesToMeters(double)}.
     */
    public SwerveModuleSimulation(
            DCMotor driveMotor,
            DCMotor steerMotor,
            double driveGearRatio,
            double steerGearRatio,
            Current driveCurrentLimit,
            Current steerCurrentLimit,
            Voltage driveFrictionVoltage,
            Voltage steerFrictionVoltage,
            Distance wheelRadius,
            MomentOfInertia steerRotationalInertia,
            double tireCoefficientOfFriction) {

        DRIVE_CURRENT_LIMIT = driveCurrentLimit;
        DRIVE_GEAR_RATIO = driveGearRatio;
        STEER_GEAR_RATIO = steerGearRatio;
        DRIVE_FRICTION_VOLTAGE = driveFrictionVoltage;
        WHEELS_COEFFICIENT_OF_FRICTION = tireCoefficientOfFriction;
        WHEEL_RADIUS = wheelRadius;

        this.driveMotorConfigs = new SimMotorConfigs(
                        driveMotor, DRIVE_GEAR_RATIO, KilogramSquareMeters.zero(), driveFrictionVoltage)
                .withStatorCurrentLimit(DRIVE_CURRENT_LIMIT)
                .withDefaultFeedForward(Volts.zero());

        SimulatedBattery.getInstance().addElectricalAppliances(this::getDriveMotorSupplyCurrent);
        this.steerMotorSim = new MapleMotorSim(
                new SimMotorConfigs(steerMotor, steerGearRatio, steerRotationalInertia, steerFrictionVoltage)
                        .withStatorCurrentLimit(steerCurrentLimit)
                        .withControllerContinousInput()
                        .withPositionVoltageController(
                                Volts.per(Degree).ofNative(8.0 / 60.0), VoltsPerRadianPerSecond.ofNative(0), false));

        this.driveWheelFinalPositionCache = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
            driveWheelFinalPositionCache.offer(driveWheelFinalPosition);
        this.steerAbsolutePositionCache = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
            steerAbsolutePositionCache.offer(steerAbsoluteFacing);
    }

    public SimMotorConfigs getDriveMotorConfigs() {
        return driveMotorConfigs;
    }

    public SimMotorConfigs getSteerMotorConfigs() {
        return steerMotorSim.getConfigs();
    }

    /**
     *
     *
     * <h2>Requests the Driving Motor to Run at a Specified Output.</h2>
     *
     * <p>Think of it as the <code>requestOutput()</code> of your physical driving motor.
     *
     * @param request the control request to apply
     */
    public void requestDriveControl(ControlRequest request) {
        this.driveMotorRequest = request;
    }

    /**
     *
     *
     * <h2>Requests the Steering Motor to Run at a Specified Output.</h2>
     *
     * <p>Think of it as the <code>requestOutput()</code> of your physical steering motor.
     *
     * @param request the control request to apply
     * @see MapleMotorSim#requestOutput(ControlRequest)
     */
    public void requestSteerControl(ControlRequest request) {
        this.steerMotorSim.requestOutput(request);
    }

    /**
     *
     *
     * <h2>Obtains the Actual Output Voltage of the Drive Motor.</h2>
     *
     * @return the actual output voltage of the drive motor
     */
    public Voltage getDriveMotorAppliedVoltage() {
        return driveMotorAppliedVoltage;
    }

    /**
     *
     *
     * <h2>Obtains the Actual Output Voltage of the Steering Motor.</h2>
     *
     * @return the actual output voltage of the steering motor
     * @see MapleMotorSim#getAppliedVoltage()
     */
    public Voltage getSteerMotorAppliedVoltage() {
        return steerMotorSim.getAppliedVoltage();
    }

    /**
     *
     *
     * <h2>Obtains the Amount of Current Supplied to the Drive Motor.</h2>
     *
     * @return the current supplied to the drive motor
     */
    public Current getDriveMotorSupplyCurrent() {
        return getDriveMotorStatorCurrent()
                .times(driveMotorAppliedVoltage.divide(
                        SimulatedBattery.getInstance().getBatteryVoltage()));
    }

    /**
     *
     *
     * <h2>Obtains the Stator current the Drive Motor.</h2>
     *
     * @return the stator current of the drive motor
     */
    public Current getDriveMotorStatorCurrent() {
        return driveMotorStatorCurrent;
    }

    /**
     *
     *
     * <h2>Obtains the Amount of Current Supplied to the Steer Motor.</h2>
     *
     * @return the current supplied to the steer motor
     * @see MapleMotorSim#getSupplyCurrent()
     */
    public Current getSteerMotorSupplyCurrent() {
        return steerMotorSim.getSupplyCurrent();
    }

    /**
     *
     *
     * <h2>Obtains the Stator current the Steer Motor.</h2>
     *
     * @return the stator current of the drive motor
     * @see MapleMotorSim#getSupplyCurrent()
     */
    public Current getSteerMotorStatorCurrent() {
        return steerMotorSim.getStatorCurrent();
    }

    /**
     *
     *
     * <h2>Obtains the Position of the Drive Encoder.</h2>
     *
     * <p>This value represents the un-geared position of the encoder, i.e., the amount of radians the drive motor's
     * encoder has rotated.
     *
     * @return the position of the drive motor's encoder (un-geared)
     */
    public Angle getDriveEncoderUnGearedPosition() {
        return getDriveWheelFinalPosition().times(DRIVE_GEAR_RATIO);
    }

    /**
     *
     *
     * <h2>Obtains the Final Position of the Wheel.</h2>
     *
     * <p>This method provides the final position of the drive encoder in terms of wheel angle.
     *
     * @return the final position of the drive encoder (wheel rotations)
     */
    public Angle getDriveWheelFinalPosition() {
        return driveWheelFinalPosition;
    }

    /**
     *
     *
     * <h2>Obtains the Speed of the Drive Encoder.</h2>
     *
     * @return the un-geared speed of the drive encoder
     */
    public AngularVelocity getDriveEncoderUnGearedSpeed() {
        return getDriveWheelFinalSpeed().times(DRIVE_GEAR_RATIO);
    }

    /**
     *
     *
     * <h2>Obtains the Final Speed of the Wheel.</h2>
     *
     * @return the final speed of the drive wheel
     */
    public AngularVelocity getDriveWheelFinalSpeed() {
        return driveWheelFinalSpeed;
    }

    /**
     *
     *
     * <h2>Obtains the Relative Position of the Steer Encoder.</h2>
     *
     * @return the relative encoder position of the steer motor
     * @see MapleMotorSim#getEncoderPosition()
     */
    public Angle getSteerRelativeEncoderPosition() {
        return getSteerAbsoluteFacing().getMeasure().times(STEER_GEAR_RATIO).plus(steerRelativeEncoderOffSet);
    }

    /**
     *
     *
     * <h2>Obtains the Speed of the Steer Relative Encoder (Geared).</h2>
     *
     * @return the speed of the steer relative encoder
     * @see MapleMotorSim#getEncoderVelocity()
     */
    public AngularVelocity getSteerRelativeEncoderSpeed() {
        return getSteerAbsoluteEncoderSpeed().times(STEER_GEAR_RATIO);
    }

    /**
     *
     *
     * <h2>Obtains the Absolute Facing of the Steer Mechanism.</h2>
     *
     * @return the absolute facing of the steer mechanism, as a {@link Rotation2d}
     */
    public Rotation2d getSteerAbsoluteFacing() {
        return steerAbsoluteFacing;
    }

    /**
     *
     *
     * <h2>Obtains the Absolute Rotational Velocity of the Steer Mechanism.</h2>
     *
     * @return the absolute angular velocity of the steer mechanism
     */
    public AngularVelocity getSteerAbsoluteEncoderSpeed() {
        return steerVelocity;
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Drive Encoder's Un-Geared Position.</h2>
     *
     * <p>The values of {@link #getCachedDriveEncoderUnGearedPositions()} are cached at each sub-tick and can be
     * retrieved using this method.
     *
     * @return an array of cached drive encoder un-geared positions
     */
    public Angle[] getCachedDriveEncoderUnGearedPositions() {
        return driveWheelFinalPositionCache.stream()
                .map(value -> value.times(DRIVE_GEAR_RATIO))
                .toArray(Angle[]::new);
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Drive Encoder's Final Position (Wheel Rotations).</h2>
     *
     * <p>The values of {@link #getDriveWheelFinalPosition()} are cached at each sub-tick and are divided by the gear
     * ratio to obtain the final wheel rotations.
     *
     * @return an array of cached drive encoder final positions (wheel rotations)
     */
    public Angle[] getCachedDriveWheelFinalPositions() {
        return driveWheelFinalPositionCache.toArray(Angle[]::new);
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Steer Relative Encoder's Position.</h2>
     *
     * <p>The values of {@link #getSteerRelativeEncoderPosition()} are cached at each sub-tick and can be retrieved
     * using this method.
     *
     * @return an array of cached steer relative encoder positions
     */
    public Angle[] getCachedSteerRelativeEncoderPositions() {
        return steerAbsolutePositionCache.stream()
                .map(absoluteFacing ->
                        absoluteFacing.getMeasure().times(STEER_GEAR_RATIO).plus(steerRelativeEncoderOffSet))
                .toArray(Angle[]::new);
    }

    /**
     *
     *
     * <h2>Obtains the Cached Readings of the Steer Absolute Positions.</h2>
     *
     * <p>The values of {@link #getSteerAbsoluteFacing()} are cached at each sub-tick and can be retrieved using this
     * method.
     *
     * @return an array of cached absolute steer positions, as {@link Rotation2d} objects
     */
    public Rotation2d[] getCachedSteerAbsolutePositions() {
        return steerAbsolutePositionCache.toArray(Rotation2d[]::new);
    }

    protected double getGrippingForceNewtons(double gravityForceOnModuleNewtons) {
        return gravityForceOnModuleNewtons * WHEELS_COEFFICIENT_OF_FRICTION;
    }

    /**
     *
     *
     * <h2>Updates the Simulation for This Module.</h2>
     *
     * <p><strong>Note:</strong> Friction forces are not simulated in this method.
     *
     * @param moduleCurrentGroundVelocityWorldRelative the current ground velocity of the module, relative to the world
     * @param robotFacing the absolute facing of the robot, relative to the world
     * @param gravityForceOnModuleNewtons the gravitational force acting on this module, in newtons
     * @return the propelling force generated by the module, as a {@link Vector2} object
     */
    public Vector2 updateSimulationSubTickGetModuleForce(
            Vector2 moduleCurrentGroundVelocityWorldRelative,
            Rotation2d robotFacing,
            double gravityForceOnModuleNewtons) {
        /* Step1: Update the steer mechanism simulation */
        updateSteerSimulation();

        /* Step2: Simulate the amount of propelling force generated by the module. */
        final double grippingForceNewtons = getGrippingForceNewtons(gravityForceOnModuleNewtons);
        final Rotation2d moduleWorldFacing = this.steerAbsoluteFacing.plus(robotFacing);
        final Vector2 propellingForce =
                getPropellingForce(grippingForceNewtons, moduleWorldFacing, moduleCurrentGroundVelocityWorldRelative);

        /* Step3: Updates and caches the encoder readings for odometry simulation. */
        updateEncoderCaches();

        return propellingForce;
    }

    /**
     *
     *
     * <h2>updates the simulation for the steer mechanism.</h2>
     */
    private void updateSteerSimulation() {
        /* update the readings of the sensor */
        steerMotorSim.update(SimulatedArena.getSimulationDt());
        this.steerAbsoluteFacing = new Rotation2d(steerMotorSim.getAngularPosition());
        this.steerVelocity = steerMotorSim.getVelocity();
    }

    /**
     *
     *
     * <h2>Calculates the amount of propelling force that the module generates.</h2>
     *
     * <p>For most of the time, that propelling force is directly applied to the drivetrain. And the drive wheel runs as
     * fast as the ground velocity
     *
     * <p>However, if the propelling force exceeds the gripping, only the max gripping force is applied. The rest of the
     * propelling force will cause the wheel to start skidding and make the odometry inaccurate.
     *
     * @param grippingForceNewtons the amount of gripping force that wheel can generate, in newtons
     * @param moduleWorldFacing the current world facing of the module
     * @param moduleCurrentGroundVelocity the current ground velocity of the module, world-reference
     * @return a vector representing the propelling force that the module generates, world-reference
     */
    private Vector2 getPropellingForce(
            double grippingForceNewtons, Rotation2d moduleWorldFacing, Vector2 moduleCurrentGroundVelocity) {
        final double driveWheelTorque = getDriveWheelTorque();
        double propellingForceNewtons = driveWheelTorque / WHEEL_RADIUS.in(Meters);
        final boolean skidding = Math.abs(propellingForceNewtons) > grippingForceNewtons;
        if (skidding) propellingForceNewtons = Math.copySign(grippingForceNewtons, propellingForceNewtons);

        final double floorVelocityProjectionOnWheelDirectionMPS = moduleCurrentGroundVelocity.getMagnitude()
                * Math.cos(moduleCurrentGroundVelocity.getAngleBetween(new Vector2(moduleWorldFacing.getRadians())));

        // if the chassis is tightly gripped on floor, the floor velocity is projected to the wheel
        this.driveWheelFinalSpeed =
                RadiansPerSecond.of(floorVelocityProjectionOnWheelDirectionMPS / WHEEL_RADIUS.in(Meters));

        // if the module is skidding
        if (skidding) {
            final AngularVelocity skiddingEquilibriumWheelSpeed = driveMotorConfigs.calculateMechanismVelocity(
                    driveMotorConfigs.calculateCurrent(
                            NewtonMeters.of(propellingForceNewtons * WHEEL_RADIUS.in(Meters))),
                    driveMotorAppliedVoltage);
            this.driveWheelFinalSpeed = driveWheelFinalSpeed.times(0.5).plus(skiddingEquilibriumWheelSpeed.times(0.5));
        }

        return Vector2.create(propellingForceNewtons, moduleWorldFacing.getRadians());
    }

    /**
     *
     *
     * <h2>Calculates the amount of torque that the drive motor can generate on the wheel.</h2>
     *
     * <p>Before calculating the torque of the motor, the output voltage of the drive motor is constrained for the
     * current limit through {@link SimMotorConfigs#constrainOutputVoltage(SimMotorState, Voltage)}.
     *
     * @return the amount of torque on the wheel by the drive motor, in Newton * Meters
     */
    private double getDriveWheelTorque() {
        driveMotorAppliedVoltage = Volts.of(
                MathUtil.applyDeadband(driveMotorAppliedVoltage.in(Volts), DRIVE_FRICTION_VOLTAGE.in(Volts), 12));

        final SimMotorState state = new SimMotorState(driveWheelFinalPosition, driveWheelFinalSpeed);
        driveMotorAppliedVoltage = driveMotorConfigs.constrainOutputVoltage(
                state, driveMotorRequest.updateSignal(driveMotorConfigs, state));

        /* calculate the stator current */
        driveMotorStatorCurrent =
                driveMotorConfigs.calculateCurrent(state.finalAngularVelocity(), driveMotorAppliedVoltage);

        /* calculate the torque generated */
        return driveMotorConfigs.calculateTorque(driveMotorStatorCurrent).in(NewtonMeters);
    }

    /** @return the current module state of this simulation module */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                MetersPerSecond.of(getDriveWheelFinalSpeed().in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters)),
                steerAbsoluteFacing);
    }

    /**
     *
     *
     * <h2>Obtains the "free spin" state of the module</h2>
     *
     * <p>The "free spin" state of a simulated module refers to its state after spinning freely for a long time under
     * the current input voltage
     *
     * @return the free spinning module state
     */
    protected SwerveModuleState getFreeSpinState() {
        return new SwerveModuleState(
                driveMotorConfigs
                                .calculateMechanismVelocity(
                                        driveMotorConfigs.calculateCurrent(driveMotorConfigs.friction),
                                        driveMotorAppliedVoltage)
                                .in(RadiansPerSecond)
                        * WHEEL_RADIUS.in(Meters),
                steerAbsoluteFacing);
    }

    /**
     *
     *
     * <h2>Cache the encoder values for high-frequency odometry.</h2>
     *
     * <p>An internal method to cache the encoder values to their queues.
     */
    private void updateEncoderCaches() {
        /* Increment of drive wheel position */
        this.driveWheelFinalPosition =
                this.driveWheelFinalPosition.plus(this.driveWheelFinalSpeed.times(SimulatedArena.getSimulationDt()));

        /* cache sensor readings to queue for high-frequency odometry */
        this.steerAbsolutePositionCache.poll();
        this.steerAbsolutePositionCache.offer(steerAbsoluteFacing);

        this.driveWheelFinalPositionCache.poll();
        this.driveWheelFinalPositionCache.offer(driveWheelFinalPosition);
    }

    /**
     *
     *
     * <h2>Obtains the theoretical speed that the module can achieve.</h2>
     *
     * @return the theoretical maximum ground speed that the module can achieve, in m/s
     */
    public LinearVelocity maximumGroundSpeed() {
        return MetersPerSecond.of(
                driveMotorConfigs.freeSpinMechanismVelocity().in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters));
    }

    /**
     *
     *
     * <h2>Obtains the theoretical maximum propelling force of ONE module.</h2>
     *
     * <p>Calculates the maximum propelling force with respect to the gripping force and the drive motor's torque under
     * its current limit.
     *
     * @param robotMass the mass of the robot
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     * @return the maximum propelling force of EACH module
     */
    public Force getTheoreticalPropellingForcePerModule(Mass robotMass, int modulesCount) {
        final double
                maxThrustNewtons =
                        driveMotorConfigs
                                        .calculateTorque(driveMotorStatorCurrent)
                                        .in(NewtonMeters)
                                / WHEEL_RADIUS.in(Meters),
                maxGrippingNewtons = 9.8 * robotMass.in(Kilograms) / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;

        return Newtons.of(Math.min(maxThrustNewtons, maxGrippingNewtons));
    }

    /**
     *
     *
     * <h2>Obtains the theatrical linear acceleration that the robot can achieve.</h2>
     *
     * <p>Calculates the maximum linear acceleration of a robot, with respect to its mass and
     * {@link #getTheoreticalPropellingForcePerModule(Mass, int)}.
     *
     * @param robotMass the mass of the robot
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     */
    public LinearAcceleration maxAcceleration(Mass robotMass, int modulesCount) {
        return getTheoreticalPropellingForcePerModule(robotMass, modulesCount)
                .times(modulesCount)
                .divide(robotMass);
    }

    /**
     *
     *
     * <h2>Stores the coefficient of friction of some common used wheels.</h2>
     */
    public enum WHEEL_GRIP {
        RUBBER_WHEEL(1.25),
        TIRE_WHEEL(1.2);

        public final double cof;

        WHEEL_GRIP(double cof) {
            this.cof = cof;
        }
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS Mark4
     * Swerve Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> getMark4(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.14;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.8,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module">SDS
     * Mark4-i Swerve Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> getMark4i(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.15;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                150.0 / 7.0,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS Mark4-n Swerve
     * Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> getMark4n(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 7.13;
                    case 2 -> 5.9;
                    case 3 -> 5.36;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                18.75,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x">WCP SwerveX Swerve Module</a>
     * for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> getSwerveX(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 7.85;
                    case 2 -> 7.13;
                    case 3 -> 6.54;
                    case 4 -> 6.56;
                    case 5 -> 5.96;
                    case 6 -> 5.46;
                    case 7 -> 5.14;
                    case 8 -> 4.75;
                    case 9 -> 4.41;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                11.3142,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x-flipped">WCP SwerveX Flipped
     * Swerve Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> getSwerveXFlipped(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 8.1;
                    case 2 -> 7.36;
                    case 3 -> 6.75;
                    case 4 -> 6.72;
                    case 5 -> 6.11;
                    case 6 -> 5.6;
                    case 7 -> 5.51;
                    case 8 -> 5.01;
                    case 9 -> 4.59;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                11.3714,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-xs">WCP SwerveXS Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6
     */
    public static Supplier<SwerveModuleSimulation> getSwerveXS(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 6.0;
                    case 2 -> 5.54;
                    case 3 -> 5.14;
                    case 4 -> 4.71;
                    case 5 -> 4.4;
                    case 6 -> 4.13;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                41.25,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2">WCP SwerveX2 Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9 <br>
     * X4 Ratios are gearRatioLevel 10-12
     */
    public static Supplier<SwerveModuleSimulation> getSwerveX2(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 6.0;
                    case 2 -> 5.54;
                    case 3 -> 5.14;
                    case 4 -> 4.71;
                    case 5 -> 4.4;
                    case 6 -> 4.13;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                41.25,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2-s">WCP SwerveX2S Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> getSwerveX2S(
            DCMotor driveMotor, DCMotor steerMotor, Current driveCurrentLimit, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 6.0;
                    case 2 -> 5.63;
                    case 3 -> 5.29;
                    case 4 -> 4.94;
                    case 5 -> 4.67;
                    case 6 -> 4.42;
                    case 7 -> 4.11;
                    case 8 -> 3.9;
                    case 9 -> 3.71;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                25.9,
                driveCurrentLimit,
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }
}
