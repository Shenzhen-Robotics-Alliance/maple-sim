# 2. Installing maple-sim

## Importing Vendordeps

Download the JSON vendordeps file and place it in the `/vendordeps` directory:

[Download JSON vendordeps file](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/maple-sim.json)

Alternatively, you can copy the following URL and [install a library online with WPILib VSCode](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html#installing-libraries):

```
https://raw.githubusercontent.com/Shenzhen-Robotics-Alliance/maple-sim/refs/heads/main/maple-sim.json
```


## Configuring Gradle Repository

In your `build.gradle` file, add the following lines to the `repositories` block:
```groovy
repositories {
    // Repository for maple-sim
    maven {
        url = uri("https://maven.pkg.github.com/Shenzhen-Robotics-Alliance/maple-sim")
        credentials {
            username = "Shenzhen-Robotics-Alliance"
            password = "\u0067\u0069\u0074\u0068\u0075\u0062\u005F\u0070\u0061\u0074\u005F\u0031\u0031\u0041\u0052\u0037\u0033\u0059\u004C\u0049\u0030\u0034\u0030\u0044\u004E\u006B\u0032\u006C\u0038\u004F\u004A\u006E\u0059\u005F\u0036\u0045\u0030\u006F\u0037\u004D\u0052\u004D\u0053\u0065\u006D\u0044\u0072\u0056\u006B\u0079\u0041\u006F\u0048\u004F\u0064\u0052\u007A\u0056\u0062\u0054\u0058\u0046\u004A\u0062\u0067\u006F\u0032\u0032\u0055\u0056\u0064\u0058\u0069\u004F\u0037\u0079\u0041\u004F\u0053\u0052\u004F\u005A\u0032\u0032\u0054\u0079\u006E\u0031\u0056\u0054\u004B\u006C\u0042"
        }
    }

    // maven local (if needed)
    mavenLocal()
}
```

If you are using AdvantageKit, you should already have a `repositories` block for it. Simply add the maple-sim repository, and your final `repositories` block should look like this:

```groovy
repositories {
    // Repository for AdvantageKit (if needed)
    maven {
        url = uri("https://maven.pkg.github.com/Mechanical-Advantage/AdvantageKit")
        credentials {
            username = "Mechanical-Advantage-Bot"
            password = "\u0067\u0068\u0070\u005f\u006e\u0056\u0051\u006a\u0055\u004f\u004c\u0061\u0079\u0066\u006e\u0078\u006e\u0037\u0051\u0049\u0054\u0042\u0032\u004c\u004a\u006d\u0055\u0070\u0073\u0031\u006d\u0037\u004c\u005a\u0030\u0076\u0062\u0070\u0063\u0051"
        }
    }

    // Repository for maple-sim (required)
    maven {
        url = uri("https://maven.pkg.github.com/Shenzhen-Robotics-Alliance/maple-sim")
        credentials {
            username = "Shenzhen-Robotics-Alliance"
            password = "\u0067\u0069\u0074\u0068\u0075\u0062\u005F\u0070\u0061\u0074\u005F\u0031\u0031\u0041\u0052\u0037\u0033\u0059\u004C\u0049\u0030\u0034\u0030\u0044\u004E\u006B\u0032\u006C\u0038\u004F\u004A\u006E\u0059\u005F\u0036\u0045\u0030\u006F\u0037\u004D\u0052\u004D\u0053\u0065\u006D\u0044\u0072\u0056\u006B\u0079\u0041\u006F\u0048\u004F\u0064\u0052\u007A\u0056\u0062\u0054\u0058\u0046\u004A\u0062\u0067\u006F\u0032\u0032\u0055\u0056\u0064\u0058\u0069\u004F\u0037\u0079\u0041\u004F\u0053\u0052\u004F\u005A\u0032\u0032\u0054\u0079\u006E\u0031\u0056\u0054\u004B\u006C\u0042"
        }
    }

    // maven local (if needed)
    mavenLocal()
}
```

> 💡 If you're using IntelliJ IDEA, remember to click the "Gradle sync" button after configuring the repository.

<div style="display:flex">
    <h3 style="width:49%"><< Prev: <a href="./1_HARDWARE_ABSTRACTIONS.md">Hardware Abstractions</a></h3>
    <h3 style="width:49%" align="right">Next: <a href="./3_USING_THE_SIMULATED_ARENA.md">Using the Simulated Arena Object</a> >></h3>
</div>