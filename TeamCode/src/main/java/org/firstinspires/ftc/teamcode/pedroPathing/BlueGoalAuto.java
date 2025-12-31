package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name = "BlueGoalAuto", group = "Autonomous")
public class BlueGoalAuto extends OpMode {

    private Follower follower;
    private DcMotorEx transferMotor, spindexerMotor, intakeMotor, flywheelMotor;
    private Servo arcLeftServo, arcRightServo, turretServo;
    private NormalizedColorSensor spindexerColorSensor, transferColorSensor;    //near intake, near shooter
    private Limelight3A limelight;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // motor values
    private double spindexerMotorValue = 0.2;
    private double intakeMotorIntakeValue = 1;
    private double transferMotorRotatingValue = -0.6;
    private double transferMotorReleaseValue = 1;
    private double flywheelMotorReleaseFar = -1;
    private double flywheelMotorReleaseClose = -0.2;
    private double spindexerRelease = 0.25;

    private double arcServoFarRight = 0.405;    //higher hood, higher arc, higher range
    private double arcServoFarLeft = 0.72;    //higher hood, higher arc, higher range
    private double arcServoCloseRight = 0.435;     //lower hood, shorter range, front triangle
    private double arcServoCloseLeft = 0.69;
    private double driveMaxPowerIntaking = 0.3;


    // revolution detection state variables
    private boolean checkingRevolution = false;
    private int revStartPos = 0;
    private final int revolutionTicks = 530;
    private final double colorThreshold = 0.0006;
    private double spindexerStartTime = 0;
    private boolean isLaunching=false;
    private boolean isIntaking=false;
    private boolean launchInProgress = false;
    private int launchStartTicks = 0;


    //sorting spindexer variables
    enum ArtifactColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    private ArrayList<ArtifactColor> spindexerQueue = new ArrayList<>();
    private ArrayList<ArtifactColor> desiredOrder = new ArrayList<>(); //2D array of possible scoring patterns depending on randomization
    private ArrayList<String> scoredPattern = new ArrayList<>();
    private boolean spindexerMoving = false;
    private boolean artifactCaptured = false;
    private int aprilTagIndex = -1;

    // Encoder tracking
    private int pocketStartTicks = 0;

    // Constants
    private final int TICKS_PER_POCKET = 130;
    private int shootStartTicks = 0;

    // Color thresholds
    private final double PURPLE_THRESHOLD = 0.001;  //checking blue min (for purple)
    private final double GREEN_THRESHOLD = 0.0015;  //checking green min


    // declaring poses
    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(145));
    private final Pose scorePreloadPose = new Pose(60, 90, Math.toRadians(145));
    private final Pose pickup1InitialPose = new Pose(45, 86, Math.toRadians(180));

    private final Pose pickup1FinalPose = new Pose(17, 86, Math.toRadians(180));
    private final Pose scoreFirstPickUpPose = new Pose(60, 75, Math.toRadians(180));
    private final Pose pickup2InitialPose = new Pose(42, 60, Math.toRadians(180));
    private final Pose pickup2FinalPose = new Pose(30, 60, Math.toRadians(180));
    private final Pose pickup3InitialPose = new Pose(40, 35, Math.toRadians(180));
    private final Pose pickup3FinalPose = new Pose(30, 30, Math.toRadians(180));

    private PathChain scorePreload, pickupArtifact1Initial, pickupArtifact1Final, scoreFirstPickUp;

    public void setSpindexerRelease() throws InterruptedException {
        if(isLaunching){
            spindexerMotor.setPower(spindexerRelease);
            transferMotor.setPower(transferMotorReleaseValue);
        }
        else{
            spindexerMotor.setPower(0);
            transferMotor.setPower(0);
            isLaunching=false;
        }
    }

    public void setIntake() throws InterruptedException {
        if(isIntaking){
            intakeMotor.setPower(intakeMotorIntakeValue);
        }
        else{
            intakeMotor.setPower(0);
            spindexerMotor.setPower(0);
            transferMotor.setPower(0);
        }
    }

   //changes spindexer and transfer motor values depending on action
    public void spindexerControls() {
        NormalizedRGBA colors = spindexerColorSensor.getNormalizedColors();

            intakeMotor.setPower(intakeMotorIntakeValue);
            if (colors.red > 0.0005 || colors.green > 0.0009 || colors.blue > 0.0009) {
                spindexerMotor.setPower(spindexerMotorValue);
                transferMotor.setPower(transferMotorRotatingValue);
            } else {
                spindexerMotor.setPower(0);
                transferMotor.setPower(0);
        }

        telemetry.addData("RED", colors.red);
        telemetry.addData("GREEN", colors.green);
        telemetry.addData("BLUE", colors.blue);
        telemetry.addData("IsIntaking", isIntaking);
        telemetry.addData("SpindexerPower", spindexerMotor.getPower());
        telemetry.addData("TransferPower", transferMotor.getPower());
    }


    //color checking
    private boolean isGreenArtifact(NormalizedRGBA c) {
        return c.green > GREEN_THRESHOLD && c.green > c.blue && c.green > c.red;
    }

    private boolean isPurpleArtifact(NormalizedRGBA c) {
        return c.blue > PURPLE_THRESHOLD && c.blue > c.red && c.blue > c.green;
    }

    private ArtifactColor detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();

        if (isGreenArtifact(c))  return ArtifactColor.GREEN;
        if (isPurpleArtifact(c)) return ArtifactColor.PURPLE;

        return ArtifactColor.UNKNOWN;
    }


    //adding desired order according to detected april tag
    private void setDesiredOrderFromAprilTag(int tagID) {
        desiredOrder.clear();

        if (tagID == 22) {
            desiredOrder.add(ArtifactColor.PURPLE);
            desiredOrder.add(ArtifactColor.GREEN);
            desiredOrder.add(ArtifactColor.PURPLE);
        } else if (tagID == 23) {
            desiredOrder.add(ArtifactColor.PURPLE);
            desiredOrder.add(ArtifactColor.PURPLE);
            desiredOrder.add(ArtifactColor.GREEN);
        } else {
            desiredOrder.add(ArtifactColor.GREEN);
            desiredOrder.add(ArtifactColor.PURPLE);
            desiredOrder.add(ArtifactColor.PURPLE);
        }
    }

    private enum ShootState {
        IDLE,
        ALIGNING,
        SHOOTING
    }
    private ShootState shootState = ShootState.IDLE;

    private void sortingSpindexer() {
        if (!isLaunching) return;

        ArtifactColor wanted = desiredOrder.isEmpty() ? ArtifactColor.UNKNOWN : desiredOrder.get(0);

        ArtifactColor transferSeen = detectColor(transferColorSensor);
        ArtifactColor intakeSeen = detectColor(spindexerColorSensor);

        switch (shootState) {

            case IDLE:
                shootState = ShootState.ALIGNING;
                break;

            case ALIGNING:
                if (transferSeen == wanted && transferSeen != ArtifactColor.UNKNOWN) {
                    // Correct color is ready → shoot
                    launchStartTicks = spindexerMotor.getCurrentPosition();
                    transferMotor.setPower(transferMotorReleaseValue);
                    spindexerMotor.setPower(spindexerRelease);
                    shootState = ShootState.SHOOTING;
                    return;
                }

                if (intakeSeen == wanted) {
                    // Desired ball is at intake → rotate clockwise until it reaches transfer
                    transferMotor.setPower(transferMotorRotatingValue);
                    spindexerMotor.setPower(-spindexerMotorValue); // rotate clockwise
                } else {
                    // Desired ball not at intake → rotate counter-clockwise one pocket
                    launchStartTicks = spindexerMotor.getCurrentPosition();
                    transferMotor.setPower(transferMotorRotatingValue); // reverse
                    spindexerMotor.setPower(spindexerMotorValue);   //rotate counter clockwise
                    shootState = ShootState.SHOOTING;
                    return;
                }
                break;

            case SHOOTING:
                int ticksMoved = Math.abs(spindexerMotor.getCurrentPosition() - launchStartTicks);
                if (ticksMoved >= TICKS_PER_POCKET) {
                    stopSpindexer();

                    ArtifactColor seenAfter = detectColor(transferColorSensor);

                    // Shot or ready
                    if (seenAfter == ArtifactColor.UNKNOWN || seenAfter == wanted) {
                        if (!desiredOrder.isEmpty()) desiredOrder.remove(0);
                        isLaunching = false;
                        shootState = ShootState.IDLE;
                    } else {
                        // Not the desired color yet → go back to aligning
                        shootState = ShootState.ALIGNING;
                    }
                }
                break;
        }
    }

    private boolean shooterHasCorrectColor() {
        ArtifactColor seen = detectColor(transferColorSensor);
        return seen != ArtifactColor.UNKNOWN && seen == desiredOrder.get(0);
    }
    private void spinUntilCorrectColor() {
        transferMotor.setPower(transferMotorRotatingValue);
        spindexerMotor.setPower(spindexerMotorValue);
    }
    private void stopSpindexer() {
        transferMotor.setPower(0);
        spindexerMotor.setPower(0);
    }


    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                .build();

        pickupArtifact1Initial = follower.pathBuilder()
                .addPath(new BezierLine(scorePreloadPose, pickup1InitialPose))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pickup1InitialPose.getHeading())
                .build();

        pickupArtifact1Final = follower.pathBuilder()
                .addPath(new BezierLine(pickup1InitialPose, pickup1FinalPose))
                .setLinearHeadingInterpolation(pickup1InitialPose.getHeading(), pickup1FinalPose.getHeading())
                .build();
        scoreFirstPickUp = follower.pathBuilder()
                .addPath(new BezierLine(pickup1FinalPose, scoreFirstPickUpPose))
                .setLinearHeadingInterpolation(pickup1FinalPose.getHeading(), scoreFirstPickUpPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() throws InterruptedException {

        switch (pathState) {
            case 0: // start flywheel and move along path
                arcLeftServo.setPosition(arcServoCloseLeft);
                arcRightServo.setPosition(arcServoCloseRight);
                flywheelMotor.setPower(flywheelMotorReleaseClose);
                turretServo.setPosition(0.5); // move turret for 0.5s
                // start path following
                follower.followPath(scorePreload);
                pathTimer.resetTimer();
                setPathState(1);
                break;

            case 1: // shooting at scorePreload
                double currentTime = pathTimer.getElapsedTimeSeconds();
                // start shooting only after follower is done
                if (!follower.isBusy() && !isLaunching) {
                    // read AprilTag once
                    if (limelight.getLatestResult() != null &&
                            limelight.getLatestResult().isValid() &&
                            !limelight.getLatestResult().getFiducialResults().isEmpty()) {

                        aprilTagIndex = limelight.getLatestResult()
                                .getFiducialResults()
                                .get(0)
                                .getFiducialId();
                    }

                    setDesiredOrderFromAprilTag(aprilTagIndex);
                    flywheelMotor.setPower(flywheelMotorReleaseClose);
                    pathTimer.resetTimer(); // optional, for next timed actions
                    isLaunching = true; // start shooting
                }

                // advance path when shooting is complete
                if (!isLaunching && !follower.isBusy()) {
                    flywheelMotor.setPower(0);
                    isIntaking = true;
                    intakeMotor.setPower(intakeMotorIntakeValue);
                    follower.followPath(pickupArtifact1Initial);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    sleep(500);
                    follower.followPath(pickupArtifact1Final, driveMaxPowerIntaking, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    sleep(300);
                    intakeMotor.setPower(0);
                    isIntaking=false;
                    follower.followPath(scoreFirstPickUp);
                    flywheelMotor.setPower(flywheelMotorReleaseClose);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    transferMotor.setPower(transferMotorReleaseValue);
                    spindexerMotor.setPower(spindexerMotorValue);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        sortingSpindexer();

        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        if (isIntaking) {
            spindexerControls();
        }
        else {
            spindexerMotor.setPower(0);
            transferMotor.setPower(0);
        }

        if (limelight.getLatestResult() != null &&
                limelight.getLatestResult().isValid() &&
                !limelight.getLatestResult().getFiducialResults().isEmpty()) {

            aprilTagIndex = limelight.getLatestResult()
                    .getFiducialResults()
                    .get(0)
                    .getFiducialId();
        }

        setDesiredOrderFromAprilTag(aprilTagIndex);

        telemetry.addData("AprilTag ID", aprilTagIndex);
        ArtifactColor spindexerColor = detectColor(spindexerColorSensor);
        ArtifactColor transferColor = detectColor(transferColorSensor);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Checking Revolution", checkingRevolution);
        telemetry.addData("RevStartPos", revStartPos);
        telemetry.addData("Spindexer Sensor Color", spindexerColor);
        telemetry.addData("Transfer Sensor Color", transferColor);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        // Servos
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        arcLeftServo = hardwareMap.get(Servo.class, "arcLeftServo");
        arcRightServo = hardwareMap.get(Servo.class, "arcRightServo");

        // Sensor
        spindexerColorSensor = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor");
        transferColorSensor = hardwareMap.get(NormalizedColorSensor.class, "transferColorSensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Encoder setup
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //limelight setup
        limelight.start();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
