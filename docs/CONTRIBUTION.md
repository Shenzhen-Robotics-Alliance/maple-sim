# How to Contribute

!!! info ""
    🙏 First, we want to express our heartfelt thanks for taking the time to read this. Your willingness to contribute means a lot to us! By supporting this project, you're helping to extend the values of FIRST and the spirit of open source. Contributors like you are essential to making this project better, and we are truly grateful for your involvement.

### Creating a fork
Before you start working on the code, you'll need to [create a fork](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/fork). **Make sure to avoid selecting the "Copy the main branch only" option.**

Once you've created your fork, pull the code locally and get started!

### Code Style Guidelines

When writing code for the project (excluding templates and examples), please adhere to the following guidelines:

!!! tip
    We use Spotless for automatic formatting. Your code will be formatted every time you compile it.

!!! success "**Things you should do**"

    ---
    - **Be a "never nester".** 
    :   See [this video](https://www.youtube.com/watch?v=CFRhGnuXG-4) for an explanation.
    
    - **Use `camelCase` for variables and `ALL_CAPS` for constants.**

    - **Ensure variable/constant names are descriptive and self-explanatory.** 
    :   e.g., `differenceBetweenGroundAndDesiredVelocityMetersPerSecond`, `ROBOT_MASS_WITH_BUMPERS`.

    - **Use [WPILib units library](https://docs.wpilib.org/pt/latest/docs/software/basic-programming/java-units.html) for configurations and APIs.**  
    :   It’s fine to use `double` for intermediate variables during computation, but make sure to use SI units and put units to the variable name (e.g., `rawMotorTorqueNewtonMeters`, `delaySeconds`).
    
    - **Add Javadocs for all public functions and constructors.**  
    :   Method Javadocs should begin with a `<h2>` title.

    - **Provide references for math/physics equations used in your code.**

!!! failure "**Things to avoid**"
    
    ---
    - ***Avoid* using `m_variable` or `k_variable` to distinguish constants from variables.**"
    
    - ***Avoid* excessively long files.**"
    :   i.e. files longer than 600 lines.

### Testing your code

To test your code, update the version name in `./project/publish.gradle`:

```groovy
// def pubVersion = '0.2.0'
// change version to:
def pubVersion = 'my-new-feature-preview'
```

Next, open the project in [IntelliJ](https://www.jetbrains.com/idea/) and click "Publish to Maven Local."

![](./media/publish%20to%20maven%20local.png){ width="30%" }

Now that the library is on your local machine, copy `./docs/vendordep/maple-sim.json` to the vendordeps directory of one of the [templates](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/main/templates) to test your code.

### Creating a PR
Once you've tested your code and confirmed that it works, you can [create a Pull Request](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/compare) towards our repo:
- If you're fixing a known bug, please create the PR against the `main` branch.
- If you're working on a new feature that already has a branch, create the PR towards that specific branch (e.g., `Shenzhen-Robotics-Alliance/maple-sim/tree/a-new-feature`).
- If you're working on a new feature that doesn't have a dedicated branch, please create the PR against the `dev` branch. Be sure to include detailed descriptions of the changes you made.

**If your PR includes API changes, it's recommended to update the documentation (`./docs/`) and example projects (`./templates/`) as part of the PR.**

### Stay in touch with us on discord!
For smoother collaboration, please [join our discord community](https://discord.com/invite/tWn45Qm6ub)!