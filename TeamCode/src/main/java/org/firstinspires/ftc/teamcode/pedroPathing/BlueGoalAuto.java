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
    private double spindexerMotorValue = 0.4;
    private double intakeMotorIntakeValue = 1;
    private double transferMotorRotatingValue = -0.8;
    private double transferMotorReleaseValue = 1;
    private double flywheelMotorReleaseFar = -1;
    private double flywheelMotorReleaseClose = -0.6;
    private double spindexerRelease = 0.25;

    private double arcServoFarRight = 0.405;    //higher hood, higher arc, higher range
    private double arcServoFarLeft = 0.72;    //higher hood, higher arc, higher range
    private double arcServoCloseRight = 0.43;     //lower hood, shorter range, front triangle
    private double arcServoCloseLeft = 0.695;
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
    private final double GREEN_THRESHOLD = 0.002;  //checking green min


    // declaring poses
    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(135));
    private final Pose readAprilTagPosition = new Pose(60, 100, Math.toRadians(125));
    private final Pose scorePreloadPosition = new Pose(75, 65, Math.toRadians(160));
    private final Pose pickup1InitialPose = new Pose(55, 65, Math.toRadians(180));

    private final Pose pickup1FinalPose = new Pose(17, 86, Math.toRadians(180));
    private final Pose scoreFirstPickUpPose = new Pose(60, 75, Math.toRadians(180));
    private final Pose pickup2InitialPose = new Pose(42, 60, Math.toRadians(180));
    private final Pose pickup2FinalPose = new Pose(30, 60, Math.toRadians(180));
    private final Pose pickup3InitialPose = new Pose(40, 35, Math.toRadians(180));
    private final Pose pickup3FinalPose = new Pose(30, 30, Math.toRadians(180));

    private PathChain readAprilTag, scorePreload, pickupArtifact1Initial, pickupArtifact1Final, scoreFirstPickUp;

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

    private void sortArtifact() throws InterruptedException {

        ArtifactColor firstWanted = desiredOrder.get(0);
        ArtifactColor secondWanted = desiredOrder.get(1);

        ArtifactColor transferSeen = detectColor(transferColorSensor);
        ArtifactColor intakeSeen = detectColor(spindexerColorSensor);

        if(transferSeen == firstWanted && intakeSeen == secondWanted){
            transferMotor.setPower(transferMotorReleaseValue);
            spindexerMotor.setPower(-spindexerRelease);
            sleep(3500);
            transferMotor.setPower(0);
            spindexerMotor.setPower(0);
        }
        else if (transferSeen == firstWanted && intakeSeen != ArtifactColor.UNKNOWN){
            transferMotor.setPower(transferMotorReleaseValue);
            spindexerMotor.setPower(spindexerRelease);
            sleep(3000);
            spindexerMotor.setPower(0);
            transferMotor.setPower(0);
        } else if (transferSeen == secondWanted && intakeSeen == firstWanted) {
            spindexerMotor.setPower(-spindexerMotorValue);
            sleep(1000);
            transferMotor.setPower(transferMotorReleaseValue);
            spindexerMotor.setPower(-spindexerRelease);
            sleep(600);
            spindexerMotor.setPower(spindexerRelease);
            sleep(2000);
            transferMotor.setPower(0);
            spindexerMotor.setPower(0);
        } else if (transferSeen != ArtifactColor.UNKNOWN && intakeSeen == firstWanted) {
            spindexerMotor.setPower(-spindexerMotorValue);
            sleep(500);
            transferMotor.setPower(transferMotorReleaseValue);
            sleep(3000);
            spindexerMotor.setPower(0);
            transferMotor.setPower(0);
        } else if (transferSeen != firstWanted && intakeSeen != firstWanted) {
            transferMotor.setPower(transferMotorRotatingValue);
            spindexerMotor.setPower(spindexerMotorValue);
            sleep(400);
            transferMotor.setPower(transferMotorReleaseValue);
            spindexerMotor.setPower(spindexerRelease);
            sleep(2000);
            transferMotor.setPower(0);
            spindexerMotor.setPower(0);
        }
        else {
            transferMotor.setPower(transferMotorReleaseValue);
            spindexerMotor.setPower(spindexerRelease);
            sleep(3000);
            transferMotor.setPower(0);
            spindexerMotor.setPower(0);
        }

    }

    private void sortingSpindexer() throws InterruptedException {
        telemetry.update();

        if (desiredOrder.isEmpty()) {
            isLaunching = false;
            return;
        }

        else if(desiredOrder.size() == 1){
            shootArtifact();
            isLaunching = false;
            return;
        }

        ArtifactColor wanted = desiredOrder.get(0);

        ArtifactColor transferSeen = detectColor(transferColorSensor);
        ArtifactColor intakeSeen = detectColor(spindexerColorSensor);

        //correct ball is at shooting position
        if (transferSeen == wanted) {
            // Correct color is ready → shoot
            shootArtifact();
        }
        else if (intakeSeen == wanted) {
            transferMotor.setPower(transferMotorRotatingValue);
            sleep(300);
            telemetry.addLine("ball shot intake");
            spindexerMotor.setPower(-spindexerRelease); // rotate clockwise
            sleep(600);
            // Desired ball is at intake → rotate clockwise until it reaches transfer
            transferMotor.setPower(-transferMotorRotatingValue);
            sleep(200);
            transferMotor.setPower(transferMotorReleaseValue);
            sleep(700);
            desiredOrder.remove(0);
            transferMotor.setPower(0);
            sleep(200);
            spindexerMotor.setPower(0);
        }
        else if (transferSeen!= ArtifactColor.UNKNOWN || intakeSeen != ArtifactColor.UNKNOWN){
            spindexerMotor.setPower(spindexerMotorValue);
            transferMotor.setPower(transferMotorRotatingValue);
            sleep(300);
            shootArtifact();
        }
        else {
            // Desired ball not at intake → rotate counter-clockwise one pocket
            spindexerMotor.setPower(spindexerMotorValue);   //rotate counter clockwise
            transferMotor.setPower(transferMotorRotatingValue);
            sleep(500);
            spindexerMotor.setPower(0);
            transferMotor.setPower(0);
            sortingSpindexer();
        }

        if (desiredOrder.isEmpty()){
            isLaunching = false;
        }
        else {
            sleep(1500);
            sortingSpindexer();
        }
    }

    private void shootArtifact() throws InterruptedException {
        // Correct color is ready → shoot
        telemetry.addLine("ball shot");
        desiredOrder.remove(0);
        transferMotor.setPower(transferMotorReleaseValue);
        spindexerMotor.setPower(spindexerRelease);
        sleep(1000);
        transferMotor.setPower(0);
        sleep(300);
        spindexerMotor.setPower(0);
        sleep(700);
    }


    public void buildPaths() {
        readAprilTag = follower.pathBuilder()
                .addPath(new BezierLine(startPose, readAprilTagPosition))
                .setLinearHeadingInterpolation(startPose.getHeading(), readAprilTagPosition.getHeading())
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(readAprilTagPosition, scorePreloadPosition))
                .setLinearHeadingInterpolation(readAprilTagPosition.getHeading(), scorePreloadPosition.getHeading())
                .build();

        pickupArtifact1Initial = follower.pathBuilder()
                .addPath(new BezierLine(scorePreloadPosition, pickup1InitialPose))
                .setLinearHeadingInterpolation(scorePreloadPosition.getHeading(), pickup1InitialPose.getHeading())
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
                turretServo.setPosition(0.95); // move turret to 0.95
                // start path following
                follower.followPath(readAprilTag);
                setPathState(1);
                break;

            case 1: // shooting at scorePreload
                // start shooting only after follower is done
                if (!follower.isBusy()) {
                    // read AprilTag once
                    if (limelight.getLatestResult() != null &&
                            limelight.getLatestResult().isValid() &&
                            !limelight.getLatestResult().getFiducialResults().isEmpty()) {

                        aprilTagIndex = limelight.getLatestResult()
                                .getFiducialResults()
                                .get(0)
                                .getFiducialId();
                    }
                }
                setDesiredOrderFromAprilTag(aprilTagIndex);

                if (pathTimer.getElapsedTimeSeconds()>1.8){
                    turretServo.setPosition(0.35); // move turret to 0.8
                    follower.followPath(scorePreload);
                    if (pathTimer.getElapsedTimeSeconds()>4){
                        sortArtifact();
                    }
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds()>4.2 && !follower.isBusy()){
                    flywheelMotor.setPower(0);
                    isIntaking=true;
                    intakeMotor.setPower(intakeMotorIntakeValue);
                    follower.followPath(pickupArtifact1Initial);
                    if (!follower.isBusy()){
                        follower.followPath(pickupArtifact1Final, driveMaxPowerIntaking, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds()>13 && !follower.isBusy()){
                    intakeMotor.setPower(0);
                    isIntaking=false;
                    follower.followPath(scoreFirstPickUp);
                    flywheelMotor.setPower(flywheelMotorReleaseClose);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    sortingSpindexer();
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

        telemetry.addData("AprilTag ID", aprilTagIndex);
        telemetry.addData("Desired List: ", desiredOrder);
        ArtifactColor spindexerColor = detectColor(spindexerColorSensor);
        ArtifactColor transferColor = detectColor(transferColorSensor);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Checking Revolution", checkingRevolution);
        telemetry.addData("RevStartPos", revStartPos);
        telemetry.addData("Intake Sensor Color", spindexerColor);
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
