package org.firstinspires.ftc.teamcode;

// General SDK libraries
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// GoBilda PinPoint libraries
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PIDDrive.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

// Java utils
import java.util.concurrent.TimeUnit;
import java.util.List;
import java.util.ArrayList;

/*
This autonomous program uses a PD controller and a motion profiler for each directional power 
to drive the robot to a target position and angle. The bot includes 4 mecanum drive wheels and the 
GoBilda Pinpoint Odometry Computer and pods.

Ports:
- Front Left Drive: front_left_drive - PORT_MUM
- Back Left Drive: back_left_drive - PORT_NUM
- Front Right Drive: front_right_drive - PORT_NUM
- Back Right Drive: back_right_drive - PORT_NUM
- GoBilda Pinpoint: pinpoint - PORT_NUM

Extra notes:
- During the initialization, the program will recalibrate the IMU, make sure the robot is stationary before running
*/

@Autonomous(name="PIDDrive", group="Linear OpMode")
public class PIDDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(); // For tracking elapsed time of the program

    // Set these acoordingly when tuning PD controllers (useXxxController should be true during normal use)
    boolean tuning = false;
    boolean useAxialController = true;
    boolean useStrafeController = true;
    boolean useYawController = true;
    
    // Define the margin of error for the target
    double marginOfErrorDistance = 2; // Inches
    double marginOfErrorAngle = 1.5; // Degrees

    // Define the time required within the target margin to be considered reached
    double timeAtTargetRequired = 0.25; // Seconds

    // Define hardware
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null; 
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private GoBildaPinpointDriver pinpoint = null;

    // PIDF Controllers and motion profiles for each directional power
    // NOTE: Tune these in the runOpMode method using the provided instructions
    // You should also set your targets in the runOpMode method
    PIDFController axialController = null;
    PIDFController strafeController = null;
    PIDFController yawController = null;
    MotionProfile axialMotionProfile = null;
    MotionProfile strafeMotionProfile = null;
    MotionProfile yawMotionProfile = null;

    // Directional and motor power variables
    double axial; // Forward or backward (axial) power
    double strafe; // Side to side (strafing) power
    double yaw; // Rotation (yaw) power
    double max; // Used to normalize motor power
    double frontLeftPower; // Power to send to the front left motor
    double frontRightPower; // Power to send to the front right motor
    double backLeftPower; // Power to send to the back left motor
    double backRightPower; // Power to send to the back right motor
    
    // Robot position variables and target variables
    Pose2D position = null; // Pose2D position from the GoBilda Pinpoint
    double positionX = 0; // X (axial) position from the GoBilda Pinpoint (inches)
    double positionY = 0; // Y (strafe) position from the GoBilda Pinpoint (inches)
    double positionAngle = 0; // Angle (yaw) from the GoBilda Pinpoint (degrees)
    int currentTargetIndex = 0; // Index of the current target in the targets list
    Pose2D currentTarget = null; // Pose2D position of the current target (targets.get(currentTargetIndex))
    double targetX = 0; // X (axial) position of the current target (motion profiler)
    double targetY = 0; // Y (strafe) position of the current target (motion profiler)
    double targetAngle = 0; // Angle (yaw) of the current target (motion profiler)
    double finalTargetX = 0; // X (axial) position of the current final target
    double finalTargetY = 0; // Y (strafe) position of the current final target
    double finalTargetAngle = 0; // Angle (yaw) of the current final target 
    double axialMotionProfilerTarget = 0; // Target for the axial motion profile
    double strafeMotionProfilerTarget = 0; // Target for the strafe motion profile
    double yawMotionProfilerTarget = 0; // Target for the yaw motion profile

    // Hardcoded list of targets (Define you targets in the runOpMode method)
    // The robot will drive to each target in the list in order
    List<Pose2D> targets = new ArrayList<>();

    // Keep track of when the target was reached to prevent overshoot and oscillation
    double timeAtTarget = -1;

    public void updateTarget() {
        // Update the current target info based on the currentTargetIndex
        currentTarget = targets.get(currentTargetIndex);

        // Set the new motion profile targets and reset PIDF controllers
        if (useAxialController) {
            targetX = currentTarget.getX(DistanceUnit.INCH);
            axialMotionProfile.reset();
            axialMotionProfile.setTarget(targetX);
            axialController.reset();
        }
        if (useStrafeController) {
            targetY = currentTarget.getY(DistanceUnit.INCH);
            strafeMotionProfile.reset();
            strafeMotionProfile.setTarget(targetY);
            strafeController.reset();
        }
        if (useYawController) {
            targetAngle = currentTarget.getHeading(AngleUnit.DEGREES);
            yawMotionProfile.reset();
            yawMotionProfile.setTarget(targetAngle);
            yawController.reset();
        }
        return;
    }

    public boolean controllersAtTarget() {
        // Check if the PIDF controller motion profilers are at their targets
        if (useAxialController) {
            if (!axialMotionProfile.atTarget() || !axialController.atTarget()) {
                return false;
            }
        }
        if (useStrafeController) {
            if (!strafeMotionProfile.atTarget() || !strafeController.atTarget()) {
                return false;
            }
        }
        if (useYawController) {
            if (!yawMotionProfile.atTarget() || !yawController.atTarget()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void runOpMode() {
        // Initialize hardware devices, check the top comment for configuration information
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive"); // Front Left Drive
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive"); // Back Left Drive
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive"); // Front Right Drive
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive"); // Back Right Drive
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"); // GoBilda Pinpoint

        // Set the direction of the motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake on stop for accuracy
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Add targets to the list, format: new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, angle);
        if (tuning) {
            if (useAxialController) {
                targets.add(new Pose2D(DistanceUnit.INCH, 36, 0, AngleUnit.DEGREES, 0)); // For tuning axial PIDF
            } else if (useStrafeController) {
                targets.add(new Pose2D(DistanceUnit.INCH, 0, 36, AngleUnit.DEGREES, 0)); // For tuning strafe PIDF
            } else if (useYawController) {
                targets.add(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 180)); // For tuning yaw PD
            }
        }
        
        targets.add(new Pose2D(DistanceUnit.INCH, 50, 20, AngleUnit.DEGREES, 0));
        targets.add(new Pose2D(DistanceUnit.INCH, 0, 50, AngleUnit.DEGREES, 0));
        targets.add(new Pose2D(DistanceUnit.INCH, 0, 50, AngleUnit.DEGREES, 90));
        targets.add(new Pose2D(DistanceUnit.INCH, 0, 50, AngleUnit.DEGREES, 0));
        targets.add(new Pose2D(DistanceUnit.INCH, 50, 80, AngleUnit.DEGREES, 0));
        targets.add(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        
        /* 
        Create PIDF controllers and motion profiles for axial and strafe power (P, I, D, F)
        - P: Proportional gain
        - I: Integral gain
        - D: Derivative gain
        - F: Feedforward (Not used in this program)

        How to tune for each directional power:
        - Set the `tuning` variable to true at the top of the program
        - Set the `useAxialController`, `useStrafeController`, and `useYawController` variables to true or false
          based on which directional power you are tuning
        - Set the I, D, and F gains of the controller you are tuning to 0
        - Set the P gain of the controller you are tuning to a value that works well for the bot
        - Mess with I and D until the bot drives to the target and doesn't overshoot
        - NOTE: You should use the yaw controller as a P controller

        - PIDFController(P, I, D, F, isYawController);
        - MotionProfile(maxVelocity, maxAcceleration, deltaTime);
        */
        if (useAxialController) {
            axialMotionProfile = new MotionProfile(3000, 2000, 0.02);
            axialMotionProfile.setTolerance(marginOfErrorDistance); // Set the distance tolerance
            axialController = new PIDFController(0.08, 0.0, 0.0, 0.0, false);
            axialController.setTolerance(marginOfErrorDistance); // Set the distance tolerance
        }
        if (useStrafeController) {
            strafeMotionProfile = new MotionProfile(3000, 2000, 0.02);
            strafeMotionProfile.setTolerance(marginOfErrorDistance); // Set the distance tolerance
            strafeController = new PIDFController(0.08, 0.0, 0.0, 0.0, false);
            strafeController.setTolerance(marginOfErrorDistance); // Set the distance tolerance
        }
        if (useYawController) {
            yawMotionProfile = new MotionProfile(1000, 2000, 0.02);
            yawMotionProfile.setTolerance(marginOfErrorAngle); // Set the angle tolerance
            yawController = new PIDFController(0.04, 0.0, 0, 0.0, true);
            yawController.setTolerance(marginOfErrorAngle); // Set the angle tolerance
        }

        // Initialize the current target to the first target in the list (currentTargetIndex is 0)
        // This will also set the PID targets
        updateTarget();

        /*
        Set the odometry pod positions relative to the point that you want the position to be measured from.
        - The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
        Left of the center is a positive number, right of center is a negative number.
        - The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
        Forward of center is a positive number, backwards is a negative number.
        */
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);

        // Set the resolution of the odometry pods, this bot uses the GoBilda "Swingarm" pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        Set the direction that each of the two odometry pods count. 
        - The X (forward) pod should increase when you move the robot forward.
        - The Y (strafe) pod should increase when you move the robot to the left.
        */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Calibrate the IMU and reset the position (Make sure the robot is stationary before running)
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // Log that the program is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press START
        waitForStart();
        runtime.reset();

        // Run until the driver presses STOP (or we hit all of the targets)
        while (opModeIsActive()) {    
            // Update the position of the robot
            pinpoint.update();
            position = pinpoint.getPosition();
            positionX = position.getX(DistanceUnit.INCH);
            positionY = position.getY(DistanceUnit.INCH);
            positionAngle = position.getHeading(AngleUnit.DEGREES);

            // If we hit the target x, y, and angle within the specified margin, don't move.
            if (controllersAtTarget()) {
                if (timeAtTarget == -1) { // If we haven't arrived at the target
                    timeAtTarget = runtime.time(); // Set the time of arrival
                }
                if (runtime.time() - timeAtTarget >= timeAtTargetRequired) { // If we have been at the target for the required time 
                    // Stop motors to prevent drift
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    
                    if (currentTargetIndex == targets.size() - 1) { // If we have reached the last target
                        break; // Break out of the loop (end the program)
                    }
                    currentTargetIndex++; // Go to the next target
                    updateTarget(); // Update the target variables
                    timeAtTarget = -1; // Reset the time at target
                    continue; // Skip the rest of this iteration
                }
                // If we are at the target, but have not been at the target for the required time, continue running
            } else {
                timeAtTarget = -1; // Reset the time at target if we aren't at the target
            }
            
            // Use the PIDF controllers for each direction
            if (useAxialController) {
                axialMotionProfilerTarget = axialMotionProfile.update(positionX);
                axial = axialController.calculate(positionX, axialMotionProfilerTarget);
            } else { axial = 0; }
            if (useStrafeController) {
                strafeMotionProfilerTarget = strafeMotionProfile.update(positionY);
                strafe = -strafeController.calculate(positionY, strafeMotionProfilerTarget);
            } else { strafe = 0; }
            if (useYawController) {
                yawMotionProfilerTarget = yawMotionProfile.update(positionAngle);
                yaw = -yawController.calculate(positionAngle, yawMotionProfilerTarget);
            } else { yaw = 0; }
            
            // Combine the PIDF controller outputs for each axis-motion to determine each wheel's power.
            frontLeftPower = axial + strafe + yaw;
            frontRightPower = axial - strafe - yaw;
            backLeftPower = axial - strafe + yaw;
            backRightPower = axial + strafe - yaw;

            // Normalize the values so no wheel power exceeds 100% (1.0)
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Show the elapsed time, robot position, and target details on the Driver Station
            telemetry.addData("Elapsed Time", runtime.time());
            telemetry.addData("X coordinate (IN)", positionX);
            telemetry.addData("Y coordinate (IN)", positionY);
            telemetry.addData("Heading angle (DEGREES)", positionAngle);
            telemetry.addData("", "");
            telemetry.addData("Target Index", currentTargetIndex);
            telemetry.addData("Target X coordinate (IN)", targetX);
            telemetry.addData("Target Y coordinate (IN)", targetY);
            telemetry.addData("Target Heading angle (DEGREES)", targetAngle);
            telemetry.addData("", "");
            telemetry.addData("Axial Power", axial);
            telemetry.addData("Strafe Power", strafe);
            telemetry.addData("Yaw Power", yaw);
            telemetry.addData("", "");
            if (useAxialController) {
                telemetry.addData("Axial Controller At Target", axialController.atTarget());
                telemetry.addData("Axial Motion Profile At Target", axialMotionProfile.atTarget());
            }
            if (useStrafeController) {
                telemetry.addData("Strafe Controller At Target", strafeController.atTarget());
                telemetry.addData("Strafe Motion Profile At Target", strafeMotionProfile.atTarget());
            }
            if (useYawController) {
                telemetry.addData("Yaw Controller At Target", yawController.atTarget());
                telemetry.addData("Yaw Motion Profile At Target", yawMotionProfile.atTarget());
            }

            // Update the Driver Station
            telemetry.update();
        }
    }

    class MotionProfile {
        private double maxVelocity, maxAcceleration, deltaTime;
        private double targetPosition;
        private double currentVelocity;
        public double measuredPos;
        public double error;
        public double tolerance = 0;

        public MotionProfile(double maxVelocity, double maxAcceleration, double deltaTime) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.deltaTime = deltaTime;
        }
        
        public void reset() {
            this.currentVelocity = 0;
            return;
        }


        public void setTarget(double targetPosition) {
            this.targetPosition = targetPosition;
            this.currentVelocity = 0;
        }
        
        public void setTolerance(double tolerance) {
            this.tolerance = tolerance;
        }

        /**
         * Updates trapezoidal motion profile using the actual measured position
         * @param measuredPos current robot position (inches)
         * @return target position for PID
         */
        public double update(double measuredPos) {
            this.measuredPos = measuredPos;
            this.error = targetPosition - measuredPos;
            double direction = Math.signum(error); // +1 forward, -1 backward

            // Distance left to target
            double distanceRemaining = Math.abs(error);

            // Max velocity we can have to still stop in time
            double maxVelForStop = Math.sqrt(2 * maxAcceleration * distanceRemaining);

            // Choose target velocity (accelerate, cruise, or decel)
            double targetVel = Math.min(maxVelocity, maxVelForStop);

            // Smoothly update velocity toward targetVel
            if (currentVelocity < targetVel) {
                currentVelocity += maxAcceleration * deltaTime;
                if (currentVelocity > targetVel) currentVelocity = targetVel;
            } else if (currentVelocity > targetVel) {
                currentVelocity -= maxAcceleration * deltaTime;
                if (currentVelocity < targetVel) currentVelocity = targetVel;
            }

            // Predict the new target position
            double newPos = measuredPos + direction * currentVelocity * deltaTime;

            return newPos;
        }

        public boolean atTarget() {
            return Math.abs(this.error) < this.tolerance;
        }
    }

    /*
    This is a slightly modified version of the PIDF controller found in FTCLib.
    https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/controller/PIDFController.java
    */
    class PIDFController {
        private double kP, kI, kD, kF;
        private boolean angularMode;

        private double targetValue;
        private double measuredValue;
        private double minIntegral, maxIntegral;
        private double lastTimeStamp;
        private double currentTimeStamp;
        private double period;

        private double positionError;
        private double totalPositionError;
        private double prevPositionError;
        private double velocityError;
        
        private double positionErrorTolerance = 1;
        private double velocityErrorTolerance = Double.POSITIVE_INFINITY;

        /**
         * This is the full constructor for the PIDF controller. Our PIDF controller
         * includes a feed-forward value which is useful for fighting friction and gravity.
         *
         * @param kP The proportional gain.
         * @param kI The integral gain.
         * @param kD The derivative gain.
         * @param kF The feed-forward gain.
         * @param angularMode true = wrap errors to [-180, 180)
         */
        public PIDFController(double kP, double kI, double kD, double kF, boolean angularMode) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.angularMode = angularMode;

            this.targetValue = 0;
            this.measuredValue = 0;
            this.minIntegral = -1.0;
            this.maxIntegral = 1.0;
            this.lastTimeStamp = 0;
            this.period = 0;

            this.positionError = this.targetValue - this.measuredValue;
            reset();

            return;
        }

        /**
         * Resets the PIDFController.
         */
        public void reset() {
            this.totalPositionError = 0;
            this.prevPositionError = 0;
            this.lastTimeStamp = 0;

            return;
        }

        /**
         * Sets the error which is considered tolerable for use with {@link #atTarget()}.
         *
         * @param positionTolerance Position error which is tolerable.
         */
        public void setTolerance(double positionErrorTolerance) {
            setTolerance(positionErrorTolerance, Double.POSITIVE_INFINITY);

            return;
        }

        /**
         * Sets the error which is considered tolerable for use with {@link #atTarget()}.
         *
         * @param positionErrorTolerance Position error which is tolerable.
         * @param velocityErrorTolerance Velocity error which is tolerable.
         */
        public void setTolerance(double positionErrorTolerance, double velocityErrorTolerance) {
            this.positionErrorTolerance = positionErrorTolerance;
            this.velocityErrorTolerance = velocityErrorTolerance;

            return;
        }

        /**
         * Sets the setpoint for the PIDFController
         *
         * @param targetValue The desired target.
         */
        public void setTarget(double targetValue) {
            this.targetValue = targetValue;
            this.positionError = this.targetValue - this.measuredValue;
            this.velocityError = (this.positionError - this.prevPositionError) / this.period;

            return;
        }

        /**
         * Returns true if the error is within the percentage of the total input range, determined by
         * {@link #setTolerance}.
         *
         * @return Whether the error is within the acceptable bounds.
         */
        public boolean atTarget() {
            return Math.abs(this.positionError) < this.positionErrorTolerance;
        }

        public double calculate(double measuredValue, double targetValue) {
            this.targetValue = targetValue;
            return calculate(measuredValue);
        }

        /**
         * Calculates the control value.
         *
         * @param pv The current measurement of the process variable.
         * @return The calculated control value.
         */
        public double calculate(double measuredValue) {
            this.measuredValue = measuredValue;

            this.currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (this.lastTimeStamp == 0) this.lastTimeStamp = this.currentTimeStamp;
            this.period = this.currentTimeStamp - this.lastTimeStamp;
            this.lastTimeStamp = this.currentTimeStamp;

            this.positionError = this.targetValue - this.measuredValue;
            if (this.angularMode) {
                // Wrap to [-180, 180)
                this.positionError = ((this.positionError + 180) % 360 + 360) % 360 - 180;
            }

            if (Math.abs(period) > 1E-6) {
                this.velocityError = (this.positionError - this.prevPositionError) / this.period;
            } else {
                this.velocityError = 0;
            }

            this.totalPositionError += this.period * (targetValue - measuredValue);
            this.totalPositionError = this.totalPositionError < this.minIntegral ? this.minIntegral :
                                    Math.min(this.maxIntegral, this.totalPositionError);

            return this.kP * this.positionError + this.kI * this.totalPositionError + 
                    this.kD * this.velocityError + this.kF * this.targetValue;
        }
    }
}