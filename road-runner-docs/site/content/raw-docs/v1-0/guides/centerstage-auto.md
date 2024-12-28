---
title: Building an Autonomous
---

# Building an Autonomous

{{< hint info >}}
This is a community guide originally written by FTC Team 6051 and updated by FTC Team 27971. Thanks for contributing to the docs!
{{< /hint >}}

After tuning, you will be ready to build your first auto routine with Roadrunner
1.0.X. Some parts of this process will feel familiar, but just like the tuning
guide, **read this page very carefully** to fully understand the logic behind
each step/declaration.

If you copy-and-paste the provided sample code, it will likely not work for your
robot, as the non-chassis actions described here are generic and will need to be
redefined for your specific mechanisms. The intent of this page is not to
provide you with runnable code out-of-the-box but instead break down the
process of writing an autonomous routine so that you may write your own. **It is
highly recommended that you code along with the creation process**.

## Step One: Imports

As with any autonomous, you will include a package statement and imports. For
this guide, we use the following imports and package:

```java
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

//RR Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.SleepAction;


//Non RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

```

Of course, you may need additional imports depending on your robot hardware, or
you may not need all of the ones included here. In nearly all cases, Android Studio
will handle these imports for you.

## Step Two: Define Auto Setup

As with all FTC autonomous modes, it is necessary to define the file as an
autonomous routine like so:
```java
@Config
@Autonomous(name = "BlueSideTestAuto", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {}
```
<!-- TODO: link to external docs -- maybe official FTC ones / book -->
If this step is confusing, we **strongly** recommend you read through the sample
Autonomous OpModes under the FTC Robot Controller, as this is not
Roadrunner-specific and will be essential to making any and all autos.

## Step Three: Instantiating Mechanism Classes
For each mechanism *not* including your drivetrain, create a new class defining
the hardware involved in the mechanism. This hardware will form the basis for
methods that return *actions*, which we will put together to make an autonomous
routine.

The following classes instantiate a `DcMotorEx`-driven, encoder-controlled, linear
lift system and a simple servo claw.
```java
// lift class
public class Lift {
    //The Motor for the lift
    private DcMotorEx liftMotor;
    //this will be explained later
    private int liftTarget;
    private int liftPose;
    private double lastTime;
    private double kP = .003, kD = .0003;

    public Lift(HardwareMap hardwareMap) {
        //make sure to put the name from your config
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //this will be explained later
        liftTarget = liftMotor.getCurrentPosition();
        liftPose = liftTarget;
        lastTime = System.nanoTime() * 1e-9;
    }
}

// claw class
public class Claw {
    private Servo claw;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }
}
```
This will ideally feel familiar, since non-RR autonomous routines also
pull from a hardware map in this way.

## Step Four: Adding Actions to Mechanisms

For each new mechanism class, we are going to add *actions*. This is where the
process becomes RR-specific, so if this is your first time designing a
1.0.X RR autonomous, **read carefully.**

Starting with the lift, you will need to implement a position control algorithm called 
a PID controller. The theory of these is covered elsewhere, like [CTRL ALT FTC](https://www.ctrlaltftc.com/).

First, you will need to create an action that will never end and will be run parallel
to the whole autonomus. This function will be the update method used in a PID.

```java
public class LiftUpdate implements Action {
    @Override
    //This method is run repeatedly
    public boolean run(@NonNull TelemetryPacket packet) {
        //This is done for the D term of te PID
        int lastPose = liftPose;

        //Done for convience. 
        liftPose = liftMotor.getCurrentPosition();

        //This is used for the P term of the PID
        int error = liftTarget - liftPose;
        //Done for the D term, this is velocity in.. ticks per nanosecond. It makes sense in this case, though the unit is odd.
        double velocity = (liftPose - lastPose) / (System.nanoTime() - lastTime);
        //This is the entiretly of the PID algoirthm. 
        double power = kP * error + kD * velocity;

        liftMotor.setPower(power);

        //This is done for the D term of te PID
        lastTime = System.nanoTime();

        //returns false to never end
        return false;
    }
}

```

We can now create a method that instantiates a `LiftUpdate` action for convenience.
```java
public Action liftUpdate(){
    return new LiftUpdate();
}
```
Now, you can do the same thing to create `OpenClaw`, and `CloseClaw` actions.
```java

// within the Claw class
public class CloseClaw implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        claw.setPosition(0.55);
        return false;
    }
}
public Action closeClaw() {
    return new CloseClaw();
}

public class OpenClaw implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        claw.setPosition(1.0);
        return false;
    }
}
public Action openClaw() {
    return new OpenClaw();
}
```
Great! Our mechanisms are now ready to access from the `runOpMode()` method.

## Step Five: `runOpMode()` and Class Instances
After the mechanism classes, but still inside the `BlueSideTestAuto` class, we add
the following:
```java
@Override
public void runOpMode() {
    // instantiate your MecanumDrive at a particular pose.
    Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    // make a Claw instance
    Claw claw = new Claw(hardwareMap);
    // make a Lift instance
    Lift lift = new Lift(hardwareMap);
}
```

{{< hint warning >}}
Make _sure_ your `MecanumDrive` is instantiated at the correct pose.
If you end up using `lineToX()`, `lineToY()`, `strafeTo()`, `splineTo()`, or any
of their variants in your code, if the initial pose is wrong, all future
movements will be thrown off.
{{< /hint >}}

## Step Six: Placehold for Vision
In some years, you may need to create your own vision pipeline to find the 
custom element your team has created. Since vision is out of the scope of
this tutorial, we are going to set a vision output like so:
```java
// vision here that outputs position
int visionOutputPosition = 1;
```
Assuming that you have vision, you will want multiple trajectories for you to
choose from.

## Step Seven: Actually Building the Actions
These trajectories only cover the surface-level of what RR has to offer,
but they do offer valuable insight into trajectory-building structure. Note that
for `lineToX()` and `lineToY()` methods, since the current heading will be used to
construct the trajectory line, the heading normally cannot be orthogonal to the
line direction. if the heading needs to remain orthogonal, you can use
`setTangent(Math.toRadians(*angle in degrees*))` to set a tangent line for the
robot to build a trajectory along.

Please also note that in general, functions like `strafeTo()`, `lineToX()`, and `lineToY()`
will generally be slower than `splineTo()` and and other spline functions because splines can move as one continuious motion.

Without further ado, we define a path for `action1`:
```java
// actionBuilder builds from the drive steps passed to it
Action action1 = drive.actionBuilder(initialPose)
        .lineToYSplineHeading(33, Math.toRadians(0))
        .waitSeconds(2)
        .setTangent(Math.toRadians(90))
        .lineToY(48)
        .setTangent(Math.toRadians(0))
        .lineToX(32)
        .strafeTo(new Vector2d(44.5, 30))
        .turn(Math.toRadians(180))
        .lineToX(47.5)
        .waitSeconds(3)
        .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5), lift.goTo(0)))
        .strafeTo(new Vector2d(48, 12))
        .build();
```
Above, you see the function `stopAndAdd()`. It is used to stop and run an action mid trajectory.
in this case, you see it used to stop, and run a `SequentialAction()` that raises the lift, opens the claw,
waits half a second to let the game element drop, then retract the lift. When this is done, the end of the
trajectory runs.

Similarly, we can make other drive actions:
```java
Action action2 = drive.actionBuilder(initialPose)
        .lineToY(37)
        .setTangent(Math.toRadians(0))
        .lineToX(18)
        .waitSeconds(3)
        .setTangent(Math.toRadians(0))
        .lineToXSplineHeading(46, Math.toRadians(180))
        .waitSeconds(3)
        .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5), lift.goTo(0)))
        .strafeTo(new Vector2d(48, 12))
        .build();
Action action3 = drive.actionBuilder(initialPose)
        .lineToYSplineHeading(33, Math.toRadians(180))
        .waitSeconds(2)
        .strafeTo(new Vector2d(46, 30))
        .waitSeconds(3)
        .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5), lift.goTo(0)))
        .strafeTo(new Vector2d(48, 12))
        .build();
```

While the current set vision result means that trajectory actions 2 and 3 will
never be run, a dynamic vision result will allow them to be run.

 ## Step Eight: Other On-Init Actions
All of the above work we did happens during the initialization of the robot. You
want all of your vision and path building to happen up here, because both of
those take a lot of time to initialize, and you don't want to lose auto runtime
to trajectory generation.

However, if you would like to add additional servo motions, you can do that by
running `Actions.runBlocking()` on the corresponding servo actions like so:
```java
// actions that need to happen on init; for instance, a claw tightening.
Actions.runBlocking(claw.closeClaw());
```

## Step Nine: The Initialization Limbo
Now, we enter the limbo between initialization completion and start. Many teams
will choose to continuously update vision during this time, and output telemetry
as shown below.
```java
while (!isStopRequested() && !opModeIsActive()) {
    int position = visionOutputPosition;
    telemetry.addData("Position during Init", position);
    telemetry.update();
}
int startPosition = visionOutputPosition;
telemetry.addData("Starting Position", startPosition);
telemetry.update();
waitForStart();
```

## Step Ten: Runtime!
We are now in the runtime! We always add the following to be able to stop the
robot if need be.
```java
if (isStopRequested()) return;
```
Now, we are going to do a simple vision-based trajectory selection as below:
```java
Action trajectoryActionChosen;
if (startPosition == 1) {
    trajectoryActionChosen = action1;
} else if (startPosition == 2) {
    trajectoryActionChosen = action2;
} else {
    trajectoryActionChosen = action3;
}
```
Once that's handled, we are all ready to run our action sequence!

We run the `lift.update()` function in parallel with the drive action. This means
that the lift will corrects it's position at all times!
```java
Actions.runBlocking(
        new ParallelAction(
                trajectoryActionChosen,
                lift.update()
        )
);
```
Congratulations! Provided everything is configured correctly, you've just
written your first autonomous in Roadrunner 1.0.X! From here, it's all
customizing to your specific use case. With <3, Anya Levin (Team #6051, Quantum
Mechanics) and Iris (Teame#27971, Null Pointer Exception)

## Final Code
For anyone interested, here is the sample autonomous all put together!

```java
package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SleepAction;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "BLUE_TEST_AUTO", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {
    public class Lift {
        private DcMotorEx liftMotor;
        private int liftTarget;
        private int liftPose;
        private double lastTime;
        private double kP = .003, kD = .0003;

        public Lift(HardwareMap hardwareMap) {
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftTarget = liftMotor.getCurrentPosition();
            liftPose = liftTarget;
            lastTime = System.nanoTime() * 1e-9;
        }

        public class LiftUpdate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int lastPose = liftPose;
                liftPose = liftMotor.getCurrentPosition();
                int error = liftTarget - liftPose;
                double power = kP * error + kD * (liftPose - lastPose) / (System.nanoTime() * 1e-9 - lastTime);
                liftMotor.setPower(power);
                lastTime = System.nanoTime();
                return false;
            }
        }

        public class LiftGoTo implements Action {
            public LiftGoTo(int target) {
                liftTarget = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return liftPose > liftTarget + 20 && liftPose < liftTarget - 20;
            }
        }

        public Action goTo(int target) {
            return new LiftGoTo(target);
        }

        public Action update() {
            return new LiftUpdate();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action action1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5), lift.goTo(0)))
                .strafeTo(new Vector2d(48, 12))
                .build();
        Action action2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3)
                .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5), lift.goTo(0)))
                .strafeTo(new Vector2d(48, 12))
                .build();
        Action action3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3)
                .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5), lift.goTo(0)))
                .strafeTo(new Vector2d(48, 12))
                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = action1;
        } else if (startPosition == 2) {
            trajectoryActionChosen = action2;
        } else {
            trajectoryActionChosen = action3;
        }

        Actions.runBlocking(
                new ParallelAction(
                        trajectoryActionChosen,
                        lift.update()
                )
        );
    }
}
```