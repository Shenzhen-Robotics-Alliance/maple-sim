# CTRE Generated Swerve Code with Maple-Sim

This is a modified version of the [CTRE Generated Swerve Code](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html), enhanced with **Maple-Sim** for better simulation capabilities.

This integration enables a realistic physics-based simulation using Maple-Sim with CTRE Swerve. You can fine-tune autonomous modes, optimize subsystems, and test advanced interactions in a simulated environment.

## Using with New Projects

To integrate **Maple-Sim** with new CTRE swerve projects:
1. Generate the `TunerConstants.java` file using Phoenix Tuner X.
2. Place the generated file in your project at `generated/TunerConstants.java`.
3. Deploy the code to your robot!

## Using with Existing Projects

For existing CTRE swerve projects, follow these steps to integrate the simulation:

---

### Prerequisites
Ensure your project was generated using **Phoenix Tuner X `2025.2.0.0+`**.

---

### Install Maple-Sim
Install the latest version of **Maple-Sim**. Refer to the [installation guide](https://shenzhen-robotics-alliance.github.io/maple-sim/installing-maple-sim/) for detailed instructions.

---

### Add the Utility Class
Download [this source file](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/CTRE%20Swerve%20with%20maple-sim/src/main/java/frc/robot/utils/simulation/MapleSimSwerveDrivetrain.java) and place it in the `src/main/java/frc/robot/utils/simulation` directory of your project.

---

### Modify the Drive Subsystem Code
Update your `subsystems/CommandSwerveDrivetrain.java` file as follows:

1. **Change the Simulation Implementation**
    Replace the existing `startSimThread` method with the code below:
    ```java
    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                // TODO: modify the following constants according to your robot
                Pounds.of(115), // robot weight
                Inches.of(30), // bumper length
                Inches.of(30), // bumper width
                DCMotor.getKrakenX60(1), // drive motor type
                DCMotor.getFalcon500(1), // steer motor type
                1.2, // wheel COF
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    ```

2. **Regulate the Module Constants**
    Modify the constructor to regulate the module constants.
    *Note: This will modify your module constants during simulation to avoid known bugs during, skips if running on a real robot.*
    ```java
    /** ... */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                // modules
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /** ... */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                // modules
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
    ```

3. **Reset the Robot Simulation Pose**
    Update the resetPose method to sync simulation and real-world poses:
    ```java
    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null)
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.05); // Wait for simulation to update
        super.resetPose(pose);
    }
    ```
