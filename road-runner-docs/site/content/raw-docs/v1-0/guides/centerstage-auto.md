---
title: Building an Autonomous
---

# Building an Autonomous

{{< hint info >}}
This is a community guide originally written by FTC Team 6051 and updated by FTC Teams 27971 & 12087.
Thanks for contributing to the docs!
{{< /hint >}}

After tuning, you will be ready to build your first auto routine with Roadrunner
1.0.X. Some parts of this process will feel familiar, but just like the tuning
guide, **read this page very carefully** to fully understand the logic behind
each step/declaration.

If you copy-and-paste the provided sample code, it will likely not work for your
robot, as the non-chassis Actions described here are generic and will need to be
redefined for your specific mechanisms. The intent of this page is not to
provide you with runnable code out-of-the-box but instead break down the
process of writing an autonomous routine so that you may write your own. **It is
highly recommended that you code along with the creation process**.

## Step One: Imports

As with any autonomous, you will include a package statement and imports. For
this guide, we use the following imports and package:

```java
package org.firstinspires.ftc.teamcode;

// RR Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;

// Non-RR imports
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
```

Of course, you may need additional imports depending on your robot hardware, or
you may not need all the ones included here. In nearly all cases, Android Studio
will handle these imports for you.

## Step Two: Define Auto Setup

As with all FTC autonomous modes, it is necessary to define the file as an
autonomous routine like so:
```java
@Config
@Autonomous(name = "BlueSideTestAuto", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {}
```
If this step is confusing, we **strongly** recommend you read through the sample
Autonomous OpModes under the FTC Robot Controller, which can be found in your SDK or 
on [the FTCRobotController GitHub](https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples), as this is not entirely
Roadrunner-specific and will be essential to making any and all autos.

## Step Three: Instantiating Mechanism Classes
For each mechanism *not* including your drivetrain, create a new class defining
the hardware involved in the mechanism. This hardware will form the basis for
methods that return *Actions*, which we will put together to make an autonomous
routine.

The following classes instantiate a `DcMotorEx`-driven, encoder-controlled, linear
lift system and a simple servo claw.

Note: In this example, we will be placing these classes directly into the autonomous file for simplicity's sake.
While this does work, for most teams it is easier to put them
in their own "Robot," "Subsystems," or "MotorControl" class. 
```java
public class Lift {
    private DcMotorEx liftMotor;
    private int liftTarget; 
    private int liftPosition;
    private int lastError = 0;
    private double lastTime;
    // The following constants are motor/mechanism-specific and will require some tuning
    private double kP = .003; 
    private double kD = .0003;

    public Lift(HardwareMap hardwareMap) {
        // Change "liftMotor" to the name of your motor
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Change this to Direction.REVERSE to reverse the motor
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD); 
        // Store the motor's current position
        liftPosition = liftMotor.getCurrentPosition();
        // Set the target to the current position
        liftTarget = liftPosition;
        // Actions.now() is a helper function that returns the current time in seconds after some arbitray epoch
        lastTime = Actions.now(); 
    }
}

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

For each new mechanism class, we are going to add *Actions*. This is where the
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

        // Save the current position of the motor
        // (to avoid unnecessary hardware accesses, and for reading in future loops)
        liftPosition = liftMotor.getCurrentPosition();

        // Calculate the error (distance from the target)
        int error = liftTarget - liftPosition;
        
        // Calculate the velocity in ticks per second
        // by dividing the change in position by the change in time.
        double errorVelocity = (error - lastError) / (Actions.now() - lastTime);
        lastTime = Actions.now(); 
        // Use our kP and kD constants to convert the input into actual motor power values,
        // and then combine them and feed that power to the motor.
        double power = kP * error + kD * errorVelocity;

        liftMotor.setPower(power);

        // This is done for the D term of the controller
        // We set the last error now, so that we can use it to calculate error velocity next loop.
        // This way, error will still be at the value read by the last loop.
        lastError = error; 

        
        // Return true to never end
        // Note: this is not typically what you would want for Actions you plan to run mid-trajectory.
        // Since this is an update function that we want to run throughout the autonomous, 
        // we are okay with it never ending.
        return true;
    }
}

```
Now that we have an action that functions as a PID controller, we must make an action that goes to a target position.

We can do this pretty easily by making an action with a paramater like such:
```java
public class LiftGoTo implements Action {
    private int target;
    public LiftGoTo(int target) {
        //This constructor takes a paramter for a position to go to and saves it
        this.target = target;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        //sets the lift instances's target to the position given as a param for the action
        liftTarget = target;

        //this is a little dense, but it returns true if the lift is within 20 ticks of the target position
        return liftPosition >= liftTarget + 20 && liftPosition <= liftTarget - 20;
    }
}
```


We can now create a methods that instantiates a `LiftUpdate` and `LiftGoTo` action for convenience.
```java
public Action goTo(int target) {
    return new LiftGoTo(target);
}

public Action update() {
    return new LiftUpdate();
}
```



Note: If you have multiple mechanisms you want to update together, you may want to combine them for convienience.
For example:
```java
public Action updateMechanisms() {
    return new ParallelAction(
            lift1.liftUpdate(),
            lift2.liftUpdate()
    );
}
```

Now, you can do the same thing to create `OpenClaw` and `CloseClaw` Actions.
<!-- TODO: demonstrate InstantActions? Lambda action creation? etc -->
```java

// within the Claw class
public class CloseClaw implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        claw.setPosition(0.55);
        // Return false to end instantly
        // In theory we would wait for the movement to end here,
        // but since we are writing to a servo,
        // in practice we have no way of knowing that the movement has ended.
        // You may want to use this in a SequentialAction with a SleepAction afterward
        // to wait for the claw to close.
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
    // Instantiate your MecanumDrive at a particular pose.
    // Change this pose to the start position of your particular autonomous.
    Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90)); 
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    // Make instances of our Claw and Lift classes, 
    // and give them hardwareMap so that they can initialize their motors and servos.
    Claw claw = new Claw(hardwareMap);
    Lift lift = new Lift(hardwareMap);
}
```

{{< hint warning >}}
Make _sure_ your `MecanumDrive` is instantiated at the correct pose, and that you consistently line your robot up to that pose.
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
line direction. If the heading needs to remain orthogonal, you can use
`setTangent(Math.toRadians(*angle in degrees*))` to set a tangent line for the
robot to build a trajectory along.

Please also note that in general, functions like `strafeTo()`, `lineToX()`, and `lineToY()`
will generally be slower than `splineTo()` and other spline functions because multiple splines can move as one continuous motion.

Without further ado, we define a path for `action1`:
<!-- TODO: everyone seems to find lineto really confusing; demonstrate strafeTo instead? 
Consider demonstrating actual splines with setTangent? -->
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
                .stopAndAdd( 
                    new SequentialAction(
                        // Move the lift to position 2000
                        // Since we defined the lift movement with an end condition,
                        // we know that it will not move on to the next action until it finishes moving.
                        lift.goTo(2000),
                        // We did not define an end condition for claw movement Actions;
                        // they end instantly. 
                        claw.openClaw(), 
                        // So we need to wait for the claw to finish moving. 
                        new SleepAction(0.5)
                    )
                .strafeTo(new Vector2d(48, 12))
                // To save a little time, we can use a displacement marker to run an action. 
                // This way, after .5 inches traveled, the lift will go down. 
                // This way, we dont have to wait for the lift to go down before we continue the auto!
                // You could also use .afterTime(seconds, Action) to do this after a given number of seconds!
                .afterDisp(.5, lift.goTo(0))
                .build();
```
Above you see the function `stopAndAdd()`.
It is one way of stopping and running an action in the middle of your autonomous.
Specifically, it waits for any movements to end, then runs its contained Actions.
Finally, after they are complete, it starts the next movement.

Similarly, we can make other drive Actions:
```java
Action action2 = drive.actionBuilder(initialPose)
        .lineToY(37)
        .setTangent(Math.toRadians(0))
        .lineToX(18)
        .waitSeconds(3)
        .setTangent(Math.toRadians(0))
        .lineToXSplineHeading(46, Math.toRadians(180))
        .waitSeconds(3)
        .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5)))
        .strafeTo(new Vector2d(48, 12))
        .afterDisp(.5, lift.goTo(0))
        .build();
Action action3 = drive.actionBuilder(initialPose)
        .lineToYSplineHeading(33, Math.toRadians(180))
        .waitSeconds(2)
        .strafeTo(new Vector2d(46, 30))
        .waitSeconds(3)
        .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5)))
        .strafeTo(new Vector2d(48, 12))
        .afterDisp(.5, lift.goTo(0))
        .build();
```

While the current set vision result means that trajectory Actions 2 and 3 will
never be run, a dynamic vision result will allow them to be run.

## Step Eight: Other On-Init Actions

All the above work we did happens during the initialization of the robot. You
want all of your vision and path building to happen up here, because both of
those take a lot of time to initialize, and you don't want to lose auto runtime
to trajectory generation.

However, if you would like to add additional servo motions, you can do that by
running `Actions.runBlocking()` on the corresponding servo Actions like so:
```java
// Actions that need to happen on init; for instance, a claw tightening.
Actions.runBlocking(claw.closeClaw());
```

## Step Nine: The Initialization Limbo
Now, we enter the limbo between initialization completion and start. Many teams
will choose to continuously update vision during this time, and output telemetry
as shown below.
```java
// while the opmode hasn't stopped or started, or in other words, while in initialization
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

We run the `lift.update()` function in parallel with the drive action.
This means that the lift will always hold its position.

Keep in mind that ParallelAction waits until all the contained Actions end before it does.
Since lift.update() never ends, this does mean that your autonomous will not end automatically;
the stop button or the 30-second timer will have to be used to stop it.

If automatically ending is important to you, you can also replace ParallelAction with RaceAction here;
RaceAction ends immediately as soon as one of the contained Actions ends, which in this case will be the trajectory.
```java
Actions.runBlocking(
        new ParallelAction(
                trajectoryActionChosen,
                lift.update()
        )
);
```
Congratulations! Provided everything is configured correctly, you've just
written your first autonomous in Roadrunner 1.0.X! From here, your next steps are
customizing it for your specific use case. With <3, Anya Levin (Team #6051, Quantum
Mechanics) and Iris (Team #27971, Null Pointer Exception)

## Final Code
Here is the sample autonomous all put together.

```java
package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

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
        private int liftPosition;
        private int lastError = 0;
        private double lastTime;
        private double kP = .003;
        private double kD = .0003;

        public Lift(HardwareMap hardwareMap) {
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftPosition = liftMotor.getCurrentPosition();
            liftTarget = liftPosition;
            lastTime = System.nanoTime() * 1e-9;
        }

        public class LiftUpdate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftPosition = liftMotor.getCurrentPosition();

                int error = liftTarget - liftPosition;

                double errorVelocity = (error - lastError) / (System.nanoTime() * 1e-9 - lastTime);
                lastTime = System.nanoTime() * 1e-9;;

                double power = kP * error + kD * errorVelocity;

                liftMotor.setPower(power);

                lastError = error;
                return true;
            }
        }


        public class LiftGoTo implements Action {
            private int target;
            public LiftGoTo(int target) {
                this.target = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftTarget = target;
                return liftPosition >= liftTarget + 20 && liftPosition <= liftTarget - 20;
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
                .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5)))
                .strafeTo(new Vector2d(48, 12))
                .afterDisp(.5, lift.goTo(0))
                .build();
        Action action2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3)
                .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5)))
                .strafeTo(new Vector2d(48, 12))
                .afterDisp(.5, lift.goTo(0))
                .build();
        Action action3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3)
                .stopAndAdd(new SequentialAction(lift.goTo(2000), claw.openClaw(), new SleepAction(.5)))
                .strafeTo(new Vector2d(48, 12))
                .afterDisp(.5, lift.goTo(0))
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