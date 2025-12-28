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
    private CRServo turretServo;
    private Servo arcLeftServo, arcRightServo;
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
    private boolean spindexerStarted = false;
    private double flywheelStartTime = 0;
    private boolean flywheelStarted = false;

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
    private final double SPINDEXER_SORT_POWER = 0.25;
    private final boolean IS_BLUE_ALLIANCE = true;

    // Color thresholds
    private final double PURPLE_THRESHOLD = 0.001;  //checking blue min (for purple)
    private final double GREEN_THRESHOLD = 0.0015;  //checking green min


    // declaring poses
    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(145));
    private final Pose scorePreloadPose = new Pose(47, 90, Math.toRadians(180));
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
                onSpindexerStartMoving();
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

    //only record color once intake is off
    private void updateIntakeSensorSafe() {
        if (spindexerMoving) return;     // ignore while moving
        if (artifactCaptured) return;    // prevent double count

        ArtifactColor detected = detectColor(spindexerColorSensor);

        if (detected != ArtifactColor.UNKNOWN) {
            spindexerQueue.add(detected);
            artifactCaptured = true;
        }
    }

    private void onSpindexerStartMoving() {
        artifactCaptured = false;
    }

    private void shootOne() throws InterruptedException {
        flywheelMotor.setPower(flywheelMotorReleaseClose);
        transferMotor.setPower(transferMotorReleaseValue);
        spindexerMotor.setPower(spindexerRelease);

        sleep(400);

        spindexerMotor.setPower(0);
        transferMotor.setPower(0);
    }


    //begins sorting balls before shooting
    private void sortBeforeShooting() throws InterruptedException{
        if (desiredOrder.isEmpty() || spindexerQueue.isEmpty()) return;

        // 🔑 GROUND TRUTH CORRECTION
        ArtifactColor shooterColor = detectColor(transferColorSensor);
        if (shooterColor != ArtifactColor.UNKNOWN) {
            spindexerQueue.set(0, shooterColor);
        }

        ArtifactColor wanted = desiredOrder.get(0);
        ArtifactColor current = spindexerQueue.get(0);

        // Correct artifact is at shooter
        if (current == wanted) {
            shootOne();
            desiredOrder.remove(0);
            spindexerQueue.remove(0);
            return;
        }
        // Wrong artifact then rotate spindexer
        else{
            transferMotor.setPower(transferMotorRotatingValue);
            spindexerMotor.setPower(spindexerMotorValue);
            sleep(700);
            transferMotor.setPower(0);
            spindexerMotor.setPower(0);
            sortBeforeShooting();
        }
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
                turretServo.setPower(-0.3);
                sleep(1500);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // run spindexer for 4 seconds
                if (!follower.isBusy()) {
                    isLaunching=true;
                    turretServo.setPower(0);
                    sortBeforeShooting();
                    flywheelMotor.setPower(0);
                    isLaunching=false;
                    isIntaking=true;
                    spindexerMoving=true;
                    //spindexerMotor.setPower(spindexerMotorValue);
                    //transferMotor.setPower(transferMotorRotatingValue);
                    intakeMotor.setPower(intakeMotorIntakeValue);
                    follower.followPath(pickupArtifact1Initial);
                    setPathState(2); // move to pickup path
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

        telemetry.addData("path state", pathState);
        telemetry.addData("Checking Revolution", checkingRevolution);
        telemetry.addData("RevStartPos", revStartPos);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        aprilTagIndex = limelight.;

        // Motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        // Servos
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
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
