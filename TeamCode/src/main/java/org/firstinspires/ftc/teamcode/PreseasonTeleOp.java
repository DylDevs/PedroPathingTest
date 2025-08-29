package org.firstinspires.ftc.teamcode;

// General SDK libraries
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// GoBilda PinPoint libraries
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// April Tag libraries
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// Java utils
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
Simple TeleOp program for the wood bot which has 4 mecanum drive wheels and the GoBilda Pinpoint chip and pods
This program uses the gamepad for manual control and has a drive to April Tag feature

Gamepad controls:
- Axial (Forward and Backward): Left-joystick Forward/Backward
- Lateral (Strafing): Left-joystick Right and Left
- Yaw (Rotation): Right-joystick Right and Left
- Drive to April Tag: Left Bumper

Ports:
- Front Left Drive: front_left_drive - PORT_MUM
- Back Left Drive: back_left_drive - PORT_NUM
- Front Right Drive: front_right_drive - PORT_NUM
- Back Right Drive: back_right_drive - PORT_NUM
- GoBilda Pinpoint: pinpoint - PORT_NUM
- Webcame: Webcam 1

Extra notes:
- During the initialization, the program will recalibrate the IMU, make sure the robot is stationary before running
*/

@TeleOp(name="PreseasonTeleOp", group="Linear OpMode")
public class PreseasonTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(); // For tracking elapsed time of the program

    // Define drive motors (mecanum)
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // Define GoBilda Pinpoint
    private GoBildaPinpointDriver pinpoint = null;

    // Constants for April Tags (Make GAIN values smaller for smoother control, or larger for a more aggressive response)
    final double DESIRED_DISTANCE = 12.0; // This is how close the camera should get to the target (inches)
    final double SPEED_GAIN = 0.02; // Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015; // Strafe Speed Control "Gain". e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN = 0.01; // Turn Control "Gain". e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.75; // Clip the approach speed to this max value
    final double MAX_AUTO_STRAFE= 0.75; // Clip the strafing speed to this max value
    final double MAX_AUTO_TURN = 0.75; // Clip the turn speed to this max value
    private static final int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.
    final double cameraExposureMS = 250; // The exposure time in milliseconds. 250 = 0.25 second
    final double cameraGain = 6; // The gain value of the camera. 6 = 6x

    // Variables for April Tags
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the April Tag detection process.
    private AprilTagDetection desiredTag; // Used to hold the data for a detected AprilTag
    boolean targetFound = false; // Set to true when an April Tag target is detected

    // Directional power variables
    double axial; // Forward or backward (axial) power
    double strafe; // Side to side (strafing) power
    double yaw; // Rotation (yaw) power

    // Motor power variables 
    double max; // Used to normalize motor power
    double frontLeftPower; // Power to send to the front left motor
    double frontRightPower; // Power to send to the front right motor
    double backLeftPower; // Power to send to the back left motor
    double backRightPower; // Power to send to the back right motor

    // Robot position variables
    Pose2D position = null; // Pose2D position from the GoBilda Pinpoint
    double positionX = 0; // X (axial) position from the GoBilda Pinpoint (inches)
    double positionY = 0; // Y (strafe) position from the GoBilda Pinpoint (inches)
    double positionAngle = 0; // Angle (yaw) from the GoBilda Pinpoint (degrees)

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

        /*
        Set the odometry pod positions relative to the point that you want the position to be measured from.
        - The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
        Left of the center is a positive number, right of center is a negative number.
        - The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
        Forward of center is a positive number, backwards is a negative number.
        */
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // Set the resolution of the odometry pods, this bot uses the GoBilda "Swingarm" pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        /*
        Set the direction that each of the two odometry pods count. 
        - The X (forward) pod should increase when you move the robot forward.
        - The Y (strafe) pod should increase when you move the robot to the left.
        */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Calibrate the IMU and reset the position (Make sure the robot is stationary before running)
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // Initialize the April Tag detector
        initializeAprilTagDetector();

        // Log that the program is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press START
        waitForStart();

        // Reset the runtime counter and clear the driver station
        runtime.reset();
        telemetry.clearAll();

        // Run until the driver presses stop
        while (opModeIsActive()) {
            // Update the position of the robot
            pinpoint.update();
            position = pinpoint.getPosition();
            positionX = position.getX(DistanceUnit.INCH);
            positionY = position.getY(DistanceUnit.INCH);
            positionAngle = position.getHeading(AngleUnit.DEGREES);

            // Search for April Tags to alert the driver if one is found
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    // Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break; // Ron't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver if we see an April Tag
            if (targetFound) {
                telemetry.addData("\n>", "Hold the left bumper to Drive to April Tag");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>", "No April Tag Detected, drive using joysticks");
            }
            telemetry.addData("", ""); // New line

            // If the left bumper is pressed and an April Tag is detected, drive to the tag
            if (gamepad1.left_bumper && targetFound) {
                // Determine heading, distance, and yaw error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn gains to calculate how we want the robot to move.
                axial = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                yaw = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                frontLeftPower = axial - strafe - yaw;
                frontRightPower = axial + strafe + yaw;
                backLeftPower = axial + strafe - yaw;
                backRightPower = axial - strafe + yaw;
            } else {
                // Use the left joystick to go forward & strafe, and right joystick to rotate.
                axial = -gamepad1.left_stick_y; // Negative value to counteract the inverted axis
                strafe = gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                frontLeftPower = axial + strafe + yaw;
                frontRightPower = axial - strafe - yaw;
                backLeftPower = axial - strafe + yaw;
                backRightPower = axial + strafe - yaw;
            }

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

            // Show elapsed time and current position on the Driver Station
            telemetry.addData("Elapsed Time", runtime.time());
            telemetry.addData("X coordinate (IN)", positionX);
            telemetry.addData("Y coordinate (IN)", positionY);
            telemetry.addData("Heading angle (DEGREES)", positionAngle);

            // Update the Driver Station
            telemetry.update();
        }
    }

    private void initializeAprilTagDetector() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Show camera status on the Driver Station
        telemetry.addData("Camera", "Waiting");
        telemetry.update();

        // Wait for the camera to be open
        while (visionPortal == null) {
            sleep(10);
        }

        // Make sure camera is streaming before we try to set the exposure controls
        while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }

        // Show camera status on the Driver Station
        telemetry.addData("Camera", "Ready");
        telemetry.update();

        // Set camera settings (edit the exposure and gain values in the config section)
        if (!isStopRequested()) {   
            // Get exposure and gain controls
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            // Set exposure mode to manual
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50); // Wait for mode change to take effect
            }

            exposureControl.setExposure((long) cameraExposureMS, TimeUnit.MILLISECONDS); // Set exposure
            sleep(20); // Wait for exposure change to take effect
            
            gainControl.setGain(cameraGain); // Set gain
            sleep(20); // Wait for gain change to take effect
        }
    }
}