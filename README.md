# Programming Conventions

## Why Conventions?

Having conventions allows us to make clean, uniform code that integrates seamlessly, even if 5 different people made different parts. It also allows for people to become familiar with the code quicker and easier, as everything is nice and consistent. It also helps to reduce errors created by differing programming styles.

## Style

### Naming

When naming things, use clear, concise names that describe what it represents. Someone should be able to look at it and instantly know what it does. Avoid making names unnecessarily long though. 

Do not use prefixes when naming variables. Oftentimes they are simply unnecessary, and if it gets confusing, use this.variable to specify member variables.

When creating a name for something, format it in accordance with this table:
|Thing|How to Format|
|---|---|
|Class/File|UpperCamelCase|
|Variable|lowerCamelCase|
|Method|lowerCamelCase|
|Parameter|lowerCamelCase|
|Constant|SCREAMING_CASE|

### Ordering

<h4>Methods</h4>

Group methods together based on what they interact with (i.e. group getVelocity() and setVelocity() together), and separate groups with comment headers.

<h4>Triggers</h4>

Triggers should be separated into two main sections marked by comment headers for each of the two controllers. Place related triggers near each other within each section (i.e. if you have two different shoot triggers, place those near one another). Beyond that, just maintain a logical order for the triggers.

<h4>Constants</h4>

Separate constants into different sub classes for each subsystem. Inside the subclass, things should generally be ordered as follows:
1. Setpoints
2. Control stuff (PID/FF, kinematics, etc.)
3. Sensor stuff (conversion factors, etc.)
4. General motor constants (current limit, inverted, etc.)
5. Physical attributes of the subsystem (wheel size, etc.)
6. Ports

Subclasses should be ordered in some logical manner. Additionally, there will be a RobotConstants sub class for universal robot values (clock speed, logging stuff, etc.).

### Comments

General rule of thumb, don't use comments. Your code should be able to explain itself. If you have some complicated math thing or other, fine. Otherwise, avoid comments

## Misc.

### Dependency Injection

Dependency injection is a technique where one object receives its dependencies from a parent object. In our case, this generally means passing subsystems. This happens from RobotContainer, and the necessary subsystems are passed into commands via the constructor. 

### Units & Units Library

Whenever representing a value that has a unit, use WPILib's [unit library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html). In general, use the following units when representing things:

- Distance: Meters
- Angle/Rotation: Radians
- Velocity:
  - Distance: Meters/s
  - Angle/Rotation: RPM
    - Radians/s should be used for any turret-like mechanism
- Acceleration:
  - Distance: Meters/s<sup>2</sup>
  - Angle/Rotation: Radians/s<sup>2</sup>
- Current: Amps
- Voltage: Volts
- Mass: Kilograms

An exception to using unit library is rotation/angle, which we use Rotation2ds for, and positions, which we use Pose2ds for.

## Example

This repository also contains an example project showcasing all of these conventions. Look [here](https://github.com/koibots8230/Programming-Conventions/tree/main/ExampleProject). 

*Note:* This project does make use of comments to highlight certain features. **Don't do that in your normal code.**
