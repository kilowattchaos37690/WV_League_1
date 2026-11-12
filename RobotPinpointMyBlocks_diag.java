package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Pinpoint + 4-bar pods myBlocks library (pose PIDF → mecanum).
 *
 * Deploy as OnBot Java — do NOT convert this file to .blk.
 * Config device name must be exactly: {@code pinpoint}
 *
 * Call {@link RobotMyBlocks#initRobot()} first (motor directions + optional motor
 * velocity PIDF). Lift/servos are gated by {@code ENABLE_LIFT_AND_SERVOS} in
 * {@link RobotMyBlocks} — not duplicated here.
 *
 * Kids blocks: initPinpoint, setPose, driveForwardOdo, strafeRightOdo,
 * turnDegreesOdo, turnToHeadingOdo, reportPose, stopDriveOdo.
 */
public class RobotPinpointMyBlocks_diag extends BlocksOpModeCompanion {

  private static final String PINPOINT_NAME = "pinpoint";

  // --- Coach: pod geometry (MEASURE) ---
  /** X (forward) pod: left of center = +, mm. */
  private static final double X_POD_OFFSET_MM = -8;
  /** Y (strafe) pod: forward of center = +, mm. */
  private static final double Y_POD_OFFSET_MM = -127;

  private static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIR =
      GoBildaPinpointDriver.EncoderDirection.FORWARD;
  private static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIR =
      GoBildaPinpointDriver.EncoderDirection.REVERSED;

  // --- Coach: pose PIDF (Pinpoint feedback → drive power) ---
  // Keep F near 0 — F*sign(e) causes end-of-move buzz / jerk when error flips.
  private static final double FWD_P = 0.05;
  private static final double FWD_I = 0.0;
  private static final double FWD_D = 0.002;
  private static final double FWD_F = 0.0;

  private static final double STR_P = 0.05;
  private static final double STR_I = 0.0;
  private static final double STR_D = 0.002;
  private static final double STR_F = 0.0;

  private static final double HEAD_P = 0.035;
  private static final double HEAD_I = 0.0;
  private static final double HEAD_D = 0.001;
  private static final double HEAD_F = 0.0;

  private static final double POS_TOLERANCE_IN = 1.5;
  private static final double HEADING_TOLERANCE_DEG = 5.0;
  /** Unused for one-shot turns (kept for turnToHeadingOdo settle). */
  private static final double HEADING_SETTLE_SEC = 0.10;
  private static final double DEFAULT_TIMEOUT_SEC = 8.0;
  /** Soft-start on each move (seconds); 0 = off. */
  private static final double POWER_RAMP_SEC = 0.05;
  /** Within this many inches of target, scale power down (reduces end jerk). */
  private static final double DECEL_DIST_IN = 2.0;
  /** Start slowing the turn when |heading error| is below this (degrees). */
  private static final double DECEL_HEADING_DEG = 45.0;
  /** Hard cap on turn power even if caller passes 0.5+ (reduces 90° overshoot). */
  private static final double TURN_MAX_POWER = 0.10;
  /**
   * Directional trim for one-shot turns (coach-tuned on carpet).
   * B (+CW) undershoots → raise TURN_CW_SCALE (e.g. 1.05–1.15).
   * X (−CCW) overshoots → lower TURN_CCW_SCALE (e.g. 0.65–0.80).
   */
  private static final double TURN_CW_SCALE = 1.0;
  private static final double TURN_CCW_SCALE = 1.0;
  /** Seconds to ramp power to 0 at end of each segment. */
  private static final double SOFT_STOP_SEC = 0.05;

  /**
   * DEBUG LADDER (change one flag at a time after forward works again):
   * 1) HOLD_HEADING=false, HOLD_CROSS_AXIS=false → pure forward/strafe axes only
   * 2) Fix TURN_CMD_SIGN using turnDegreesOdo alone (±1)
   * 3) HOLD_HEADING=true, then HOLD_CROSS_AXIS=true
   */
  private static final boolean HOLD_CROSS_AXIS = true;
  private static final boolean HOLD_HEADING = true;
  /**
   * Maps "need more CCW heading" → mecanum turn command.
   * If turnDegreesOdo(+90) spins LEFT instead of RIGHT, flip +1 ↔ -1.
   */
  private static final double TURN_CMD_SIGN = -1.0;
  /** If strafeRightOdo goes left instead of right, set to -1. */
  private static final double STRAFE_CMD_SIGN = -1.0;
  /**
   * When true, pose/turn loops write live DS telemetry (can slow the loop slightly).
   * Toggle from diag TeleOp via {@link #setDebugPoseDrive(boolean)}.
   */
  private static boolean DEBUG_POSE_DRIVE = false;
  private static final boolean USE_MOTOR_VELOCITY_PID = false;

  /** Last relative-turn snapshot (for TeleOp diag after a blocking move). */
  private static String lastTurnStopReason = "-";
  private static double lastTurnProgCcw = 0;
  private static double lastTurnTargetCcw = 0;
  private static double lastTurnErr = 0;
  private static double lastTurnCmd = 0;
  private static double lastTurnAbsH = 0;
  private static double lastTurnElapsedSec = 0;

  private static GoBildaPinpointDriver pinpoint;
  private static DcMotorEx left_front;
  private static DcMotorEx left_back;
  private static DcMotorEx right_front;
  private static DcMotorEx right_back;
  private static boolean ready = false;

  private static final Pidf fwdPidf = new Pidf(FWD_P, FWD_I, FWD_D, FWD_F);
  private static final Pidf strPidf = new Pidf(STR_P, STR_I, STR_D, STR_F);
  private static final Pidf headPidf = new Pidf(HEAD_P, HEAD_I, HEAD_D, HEAD_F);

  /** Call from Java LinearOpModes so myBlocks see this OpMode's hardware. */
  public static void bindFromLinearOpMode(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode op) {
    opMode = op;
    linearOpMode = op;
    hardwareMap = op.hardwareMap;
    telemetry = op.telemetry;
    gamepad1 = op.gamepad1;
    gamepad2 = op.gamepad2;
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Init Pinpoint (4-bar). Config name: pinpoint. Robot must be still.",
      tooltip = "Call after RobotMyBlocks.initRobot()")
  public static void initPinpoint() {
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
    pinpoint.setEncoderDirections(X_ENCODER_DIR, Y_ENCODER_DIR);
    pinpoint.resetPosAndIMU();

    left_front = hardwareMap.get(DcMotorEx.class, "left_front");
    left_back = hardwareMap.get(DcMotorEx.class, "left_back");
    right_front = hardwareMap.get(DcMotorEx.class, "right_front");
    right_back = hardwareMap.get(DcMotorEx.class, "right_back");

    ready = true;
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Read Pinpoint into pose (call often in TeleOp loops)")
  public static void updatePose() {
    ensureReady();
    pinpoint.update();
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Set field pose inches / degrees (start of Auto)",
      parameterLabels = {"X inches", "Y inches", "Heading deg"})
  public static void setPose(double xIn, double yIn, double headingDeg) {
    ensureReady();
    pinpoint.setPosition(
        new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg));
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Enable/disable live pose+turn telemetry during Auto moves",
      parameterLabels = {"Debug on?"})
  public static void setDebugPoseDrive(boolean on) {
    DEBUG_POSE_DRIVE = on;
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Write X/Y/H + raw pod ticks to telemetry (hand-push check)")
  public static void reportPose() {
    ensureReady();
    pinpoint.update();
    if (telemetry != null) {
      // Field frame at setPose: +X = robot forward, +Y = robot left, +H = CCW
      telemetry.addData("Pinpoint X (in)", "%.2f", pinpoint.getPosX(DistanceUnit.INCH));
      telemetry.addData("Pinpoint Y (in)", "%.2f", pinpoint.getPosY(DistanceUnit.INCH));
      telemetry.addData("Pinpoint H (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
      telemetry.addData("raw enc X/Y", "%d / %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
      telemetry.addData(
          "Pinpoint status",
          pinpoint.getDeviceStatus() != null ? pinpoint.getDeviceStatus().toString() : "?");
      telemetry.addData(
          "Expect", "push fwd→X↑ & encX↑; push left→Y↑ & encY↑; spin CCW→H↑");
      telemetry.addData("debugPoseDrive", DEBUG_POSE_DRIVE);
      telemetry.addData(
          "signs", "TURN=%s STR=%s", String.valueOf(TURN_CMD_SIGN), String.valueOf(STRAFE_CMD_SIGN));
    }
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Write last turnDegreesOdo snapshot (stop reason, prog, err, cmd)")
  public static void reportLastTurn() {
    if (telemetry == null) {
      return;
    }
    telemetry.addData("lastTurn stop", lastTurnStopReason);
    telemetry.addData(
        "lastTurn prog/target CCW", "%.1f / %.1f", lastTurnProgCcw, lastTurnTargetCcw);
    telemetry.addData("lastTurn err/cmd", "%.1f / %.2f", lastTurnErr, lastTurnCmd);
    telemetry.addData("lastTurn absH (deg)", "%.1f", lastTurnAbsH);
    telemetry.addData("lastTurn elapsed (s)", "%.2f", lastTurnElapsedSec);
    telemetry.addData(
        "Read",
        "stop=passed → crossed target; settle → in tol; timeout → never settled; "
            + "if prog past target then err flips → unwind bug");
  }

  @ExportToBlocks(heading = "Pinpoint", comment = "X inches (updatePose in loop)")
  public static double getXIn() {
    ensureReady();
    return pinpoint.getPosX(DistanceUnit.INCH);
  }

  @ExportToBlocks(heading = "Pinpoint", comment = "Y inches (updatePose in loop)")
  public static double getYIn() {
    ensureReady();
    return pinpoint.getPosY(DistanceUnit.INCH);
  }

  @ExportToBlocks(heading = "Pinpoint", comment = "Heading degrees (updatePose in loop)")
  public static double getHeadingDeg() {
    ensureReady();
    return pinpoint.getHeading(AngleUnit.DEGREES);
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Drive forward (+) / back (−) inches — Pinpoint pose PIDF + heading hold",
      parameterLabels = {"Inches", "Max power 0..1"})
  public static void driveForwardOdo(double inches, double maxPower) {
    moveRobotRelative(inches, 0.0, maxPower);
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Strafe right (+) / left (−) inches — Pinpoint pose PIDF + heading hold",
      parameterLabels = {"Inches", "Max power 0..1"})
  public static void strafeRightOdo(double inches, double maxPower) {
    moveRobotRelative(0.0, inches, maxPower);
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Spin right (+) / left (−) degrees — relative (no 180° wrap)",
      parameterLabels = {"Degrees", "Max power 0..1"})
  public static void turnDegreesOdo(double degrees, double maxPower) {
    // One-shot relative turn (|degrees| ≤ 180):
    // +degrees = right/CW → targetH = startH - degrees (Pinpoint +H is CCW).
    // Drive only toward the target; never reverse. That stops end-of-move
    // oscillation / 270° unwind. Stop on first entry into heading tolerance.
    ensureReady();
    ensureDriveReady();
    pinpoint.update();
    // Apply CW/CCW trim so a requested ±90 lands near ±90 on the mat.
    double scaledDegrees =
        degrees >= 0.0 ? degrees * TURN_CW_SCALE : degrees * TURN_CCW_SCALE;
    double startH = pinpoint.getHeading(AngleUnit.DEGREES);
    double targetH = normalizeDeg(startH - scaledDegrees);
    double initialErr = angleErrorDeg(targetH, startH); // + => need CCW
    double progressedCcw = 0.0;
    double targetCcw = -scaledDegrees;
    double pMax = Math.min(Math.abs(maxPower), TURN_MAX_POWER);
    headPidf.reset();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime telemTimer = new ElapsedTime();
    double lastTurn = 0;
    double err = initialErr;
    double h = startH;
    double lastH = startH;
    String stopReason = "timeout";

    lastTurnStopReason = "running";
    lastTurnTargetCcw = targetCcw;

    while (opModeIsActiveCompat() && timer.seconds() < DEFAULT_TIMEOUT_SEC) {
      pinpoint.update();
      h = pinpoint.getHeading(AngleUnit.DEGREES);
      progressedCcw += wrapDeg(h - lastH);
      lastH = h;

      err = angleErrorDeg(targetH, h);
      double absErr = Math.abs(err);

      // Close enough — stop immediately (no settle hunt that re-kicks PID).
      if (absErr <= HEADING_TOLERANCE_DEG) {
        lastTurn = 0;
        setMecanum(0, 0, 0);
        stopReason = "settle";
        break;
      }

      // Crossed target (error flipped) — stop; do not reverse/oscillate.
      // Require ~85% of the turn first so noise cannot stop B at 60–70°.
      if (Math.abs(scaledDegrees) > 1e-6
          && Math.abs(progressedCcw) > 0.85 * Math.abs(scaledDegrees)
          && Math.signum(err) != 0
          && Math.signum(err) != Math.signum(initialErr)) {
        lastTurn = 0;
        setMecanum(0, 0, 0);
        stopReason = "crossed_target";
        break;
      }

      // Runaway safety.
      if (Math.abs(targetCcw) > 1e-6 && progressedCcw / targetCcw >= 1.20) {
        lastTurn = 0;
        setMecanum(0, 0, 0);
        stopReason = "passed_target";
        break;
      }

      double approach =
          DECEL_HEADING_DEG <= 0 ? 1.0 : Math.min(1.0, absErr / DECEL_HEADING_DEG);
      double cmd =
          TURN_CMD_SIGN
              * headPidf.update(err, pMax)
              * rampScale(timer.seconds());
      // Apply approach to the clip limit so it directly caps speed
      // starting at DECEL_HEADING_DEG out, regardless of PID magnitude.
      double limit = pMax * approach;
      lastTurn = Range.clip(cmd, -limit, limit);
      setMecanum(0, 0, lastTurn);
      if (DEBUG_POSE_DRIVE && telemetry != null && telemTimer.milliseconds() > 100) {
        telemTimer.reset();
        writeTurnTelemetry(progressedCcw, targetCcw, err, lastTurn, h, "driving", timer.seconds());
      }
      idleCompat();
    }
    stopMecanum();
    snapshotLastTurn(stopReason, progressedCcw, targetCcw, err, lastTurn, h, timer.seconds());
    if (telemetry != null) {
      reportLastTurn();
      reportPose();
      telemetry.update();
    }
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Turn to absolute field heading (degrees) — Pinpoint heading PIDF",
      parameterLabels = {"Heading deg", "Max power 0..1"})
  public static void turnToHeadingOdo(double headingDeg, double maxPower) {
    ensureReady();
    ensureDriveReady();
    double pMax = Math.min(Math.abs(maxPower), TURN_MAX_POWER);
    headPidf.reset();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime settle = new ElapsedTime();
    boolean inTol = false;
    double lastTurn = 0;
    while (opModeIsActiveCompat() && timer.seconds() < DEFAULT_TIMEOUT_SEC) {
      pinpoint.update();
      double h = pinpoint.getHeading(AngleUnit.DEGREES);
      double err = angleErrorDeg(headingDeg, h);
      double absErr = Math.abs(err);

      if (absErr < HEADING_TOLERANCE_DEG) {
        if (!inTol) {
          inTol = true;
          settle.reset();
        }
        lastTurn = 0;
        setMecanum(0, 0, 0);
        if (settle.seconds() >= HEADING_SETTLE_SEC) {
          break;
        }
        idleCompat();
        continue;
      }
      inTol = false;

      double approach =
          DECEL_HEADING_DEG <= 0 ? 1.0 : Math.min(1.0, absErr / DECEL_HEADING_DEG);
      double limit = pMax * approach;
      lastTurn =
          Range.clip(
              TURN_CMD_SIGN
                  * headPidf.update(err, pMax)
                  * rampScale(timer.seconds()),
              -limit,
              limit);
      setMecanum(0, 0, lastTurn);
      maybeDebugTurn(err, lastTurn, h, headingDeg);
      idleCompat();
    }
    stopMecanum();
  }

  @ExportToBlocks(heading = "Pinpoint", comment = "Stop drive motors after Pinpoint moves")
  public static void stopDriveOdo() {
    stopMecanum();
    if (left_front != null) {
      setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
  }

  @ExportToBlocks(
      heading = "Pinpoint",
      comment = "Open-loop mecanum for N seconds (no pose PID) — motor sanity check",
      parameterLabels = {"Forward", "Strafe", "Turn", "Seconds"})
  public static void drivePowerTimed(
      double forward, double strafe, double turn, double seconds) {
    ensureReady();
    ensureDriveReady();
    ElapsedTime timer = new ElapsedTime();
    while (opModeIsActiveCompat() && timer.seconds() < Math.abs(seconds)) {
      setMecanum(forward, strafe, turn);
      if (DEBUG_POSE_DRIVE && telemetry != null) {
        pinpoint.update();
        telemetry.addData("openLoop fwd/str/turn", "%.2f / %.2f / %.2f", forward, strafe, turn);
        telemetry.addData(
            "pose X/Y/H",
            "%.2f / %.2f / %.1f",
            pinpoint.getPosX(DistanceUnit.INCH),
            pinpoint.getPosY(DistanceUnit.INCH),
            pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();
      }
      idleCompat();
    }
    stopMecanum();
  }

  // -------------------------------------------------------------------------
  // Internals
  // -------------------------------------------------------------------------

  /**
   * Robot-relative translate: +fwdIn forward, +rightIn to the right (inches).
   * With HOLD_HEADING/HOLD_CROSS_AXIS false (debug defaults), only the commanded
   * axis is driven — same idea as when forward worked before turn tuning.
   */
  private static void moveRobotRelative(double fwdIn, double rightIn, double maxPower) {
    ensureReady();
    ensureDriveReady();
    pinpoint.update();
    double startX = pinpoint.getPosX(DistanceUnit.INCH);
    double startY = pinpoint.getPosY(DistanceUnit.INCH);
    double startH = pinpoint.getHeading(AngleUnit.DEGREES);
    double pMax = Math.abs(maxPower);

    // Commanded axes always on; cross-axis only if HOLD_CROSS_AXIS.
    boolean regulateFwd = Math.abs(fwdIn) > 1e-9 || (HOLD_CROSS_AXIS && Math.abs(rightIn) > 1e-9);
    boolean regulateRight = Math.abs(rightIn) > 1e-9 || (HOLD_CROSS_AXIS && Math.abs(fwdIn) > 1e-9);

    fwdPidf.reset();
    strPidf.reset();
    headPidf.reset();

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime telemTimer = new ElapsedTime();
    double lastFwd = 0;
    double lastStr = 0;
    double lastTurn = 0;
    while (opModeIsActiveCompat() && timer.seconds() < DEFAULT_TIMEOUT_SEC) {
      pinpoint.update();
      double x = pinpoint.getPosX(DistanceUnit.INCH);
      double y = pinpoint.getPosY(DistanceUnit.INCH);
      double h = pinpoint.getHeading(AngleUnit.DEGREES);

      double dx = x - startX;
      double dy = y - startY;
      double hRad = Math.toRadians(startH);
      double cos = Math.cos(hRad);
      double sin = Math.sin(hRad);
      // Pinpoint: +X forward, +Y left → robot +right = −left
      double traveledFwd = dx * cos + dy * sin;
      double traveledLeft = -dx * sin + dy * cos;
      double traveledRight = -traveledLeft;

      double errFwd = fwdIn - traveledFwd;
      double errRight = rightIn - traveledRight;
      double errH = HOLD_HEADING ? angleErrorDeg(startH, h) : 0.0;

      boolean done =
          (!regulateFwd || Math.abs(errFwd) < POS_TOLERANCE_IN)
              && (!regulateRight || Math.abs(errRight) < POS_TOLERANCE_IN)
              && (!HOLD_HEADING || Math.abs(errH) < HEADING_TOLERANCE_DEG);

      // One-shot: crossed the commanded distance — stop; do not reverse/oscillate.
      boolean crossedFwd =
          regulateFwd
              && Math.abs(fwdIn) > 1e-9
              && Math.abs(traveledFwd) > 0.40 * Math.abs(fwdIn)
              && Math.signum(errFwd) != 0
              && Math.signum(errFwd) != Math.signum(fwdIn);
      boolean crossedRight =
          regulateRight
              && Math.abs(rightIn) > 1e-9
              && Math.abs(traveledRight) > 0.40 * Math.abs(rightIn)
              && Math.signum(errRight) != 0
              && Math.signum(errRight) != Math.signum(rightIn);
      if (done || crossedFwd || crossedRight) {
        break;
      }

      double scale = rampScale(timer.seconds());
      double approach = approachScale(errFwd, errRight, regulateFwd, regulateRight);
      double fwd = regulateFwd ? fwdPidf.update(errFwd, pMax) * scale * approach : 0.0;
      double strafe =
          regulateRight
              ? STRAFE_CMD_SIGN * strPidf.update(errRight, pMax) * scale * approach
              : 0.0;
      double turn =
          HOLD_HEADING ? TURN_CMD_SIGN * headPidf.update(errH, pMax) * scale * approach : 0.0;
      // Never reverse the commanded axis (kills end buzz).
      if (regulateFwd && Math.signum(fwdIn) != 0 && Math.signum(fwd) != 0 && Math.signum(fwd) != Math.signum(fwdIn)) {
        fwd = 0;
      }
      if (regulateRight
          && Math.signum(rightIn) != 0
          && Math.signum(strafe) != 0
          && Math.signum(strafe) != Math.signum(STRAFE_CMD_SIGN * rightIn)) {
        strafe = 0;
      }
      lastFwd = Range.clip(fwd, -pMax, pMax);
      lastStr = Range.clip(strafe, -pMax, pMax);
      lastTurn = Range.clip(turn, -pMax, pMax);
      setMecanum(lastFwd, lastStr, lastTurn);

      if (DEBUG_POSE_DRIVE && telemetry != null && telemTimer.milliseconds() > 100) {
        telemTimer.reset();
        telemetry.addData("errFwd/Right/H", "%.2f / %.2f / %.1f", errFwd, errRight, errH);
        telemetry.addData("cmd fwd/str/turn", "%.2f / %.2f / %.2f", lastFwd, lastStr, lastTurn);
        telemetry.addData("pose X/Y/H", "%.2f / %.2f / %.1f", x, y, h);
        telemetry.update();
      }
      idleCompat();
    }
    stopMecanum();
  }

  private static double rampScale(double seconds) {
    if (POWER_RAMP_SEC <= 0) {
      return 1.0;
    }
    return Math.min(1.0, seconds / POWER_RAMP_SEC);
  }

  /** Slow down when close to the commanded target (primary axis error). */
  private static double approachScale(
      double errFwd, double errRight, boolean regulateFwd, boolean regulateRight) {
    if (DECEL_DIST_IN <= 0) {
      return 1.0;
    }
    double primary = 0;
    if (regulateFwd) {
      primary = Math.max(primary, Math.abs(errFwd));
    }
    if (regulateRight) {
      primary = Math.max(primary, Math.abs(errRight));
    }
    if (primary >= DECEL_DIST_IN) {
      return 1.0;
    }
    return Math.max(0.15, primary / DECEL_DIST_IN);
  }

  private static void softStop() {
    softStopFrom(0, 0, 0);
  }

  /** Ramp last commanded powers down so BRAKE mode does not snap the chassis. */
  private static void softStopFrom(double fwd, double strafe, double turn) {
    if (left_front == null) {
      return;
    }
    if (SOFT_STOP_SEC <= 0 || (Math.abs(fwd) < 0.02 && Math.abs(strafe) < 0.02 && Math.abs(turn) < 0.02)) {
      stopMecanum();
      return;
    }
    ElapsedTime t = new ElapsedTime();
    while (opModeIsActiveCompat() && t.seconds() < SOFT_STOP_SEC) {
      double s = 1.0 - (t.seconds() / SOFT_STOP_SEC);
      setMecanum(fwd * s, strafe * s, turn * s);
      idleCompat();
    }
    stopMecanum();
  }

  private static void maybeDebugTurn(double err, double turn, double h, double target) {
    if (!DEBUG_POSE_DRIVE || telemetry == null) {
      return;
    }
    telemetry.addData("turn errH/cmd", "%.1f / %.2f", err, turn);
    telemetry.addData("pose H", "%.1f → %.1f", h, target);
    telemetry.update();
  }

  private static void writeTurnTelemetry(
      double progCcw,
      double targetCcw,
      double err,
      double cmd,
      double absH,
      String phase,
      double elapsedSec) {
    telemetry.addData("turn phase", phase);
    telemetry.addData("turn prog/target CCW", "%.1f / %.1f", progCcw, targetCcw);
    telemetry.addData("turn err/cmd", "%.1f / %.2f", err, cmd);
    telemetry.addData("turn absH (deg)", "%.1f", absH);
    telemetry.addData("turn elapsed (s)", "%.2f", elapsedSec);
    telemetry.addData("TURN_CMD_SIGN", TURN_CMD_SIGN);
    telemetry.update();
  }

  private static void snapshotLastTurn(
      String reason,
      double progCcw,
      double targetCcw,
      double err,
      double cmd,
      double absH,
      double elapsedSec) {
    lastTurnStopReason = reason;
    lastTurnProgCcw = progCcw;
    lastTurnTargetCcw = targetCcw;
    lastTurnErr = err;
    lastTurnCmd = cmd;
    lastTurnAbsH = absH;
    lastTurnElapsedSec = elapsedSec;
  }

  private static void ensureReady() {
    if (!ready) {
      initPinpoint();
    }
  }

  /** Prefer {@link RobotMyBlocks#initRobot()} in OpModes (directions + optional motor PIDF). */
  private static void ensureDriveReady() {
    if (left_front == null) {
      left_front = hardwareMap.get(DcMotorEx.class, "left_front");
      left_back = hardwareMap.get(DcMotorEx.class, "left_back");
      right_front = hardwareMap.get(DcMotorEx.class, "right_front");
      right_back = hardwareMap.get(DcMotorEx.class, "right_back");
    }
    setDriveRunMode(
        USE_MOTOR_VELOCITY_PID
            ? DcMotor.RunMode.RUN_USING_ENCODER
            : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  private static void setDriveRunMode(DcMotor.RunMode mode) {
    left_front.setMode(mode);
    left_back.setMode(mode);
    right_front.setMode(mode);
    right_back.setMode(mode);
  }

  /** Same mecanum mix as RobotMyBlocks.driveArcade (no TeleOp stick shaping). */
  private static void setMecanum(double forward, double strafe, double turn) {
    double fl = forward + strafe + turn;
    double bl = forward - strafe + turn;
    double fr = forward - strafe - turn;
    double br = forward + strafe - turn;
    double max =
        Math.max(
            1.0,
            Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
    left_front.setPower(fl / max);
    left_back.setPower(bl / max);
    right_front.setPower(fr / max);
    right_back.setPower(br / max);
  }

  private static void stopMecanum() {
    if (left_front != null) {
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
    }
  }

  private static double wrapDeg(double deg) {
    while (deg > 180.0) {
      deg -= 360.0;
    }
    while (deg < -180.0) {
      deg += 360.0;
    }
    return deg;
  }

  private static double normalizeDeg(double deg) {
    return wrapDeg(deg);
  }

  private static double angleErrorDeg(double targetDeg, double currentDeg) {
    return wrapDeg(targetDeg - currentDeg);
  }

  private static boolean opModeIsActiveCompat() {
    return linearOpMode == null || linearOpMode.opModeIsActive();
  }

  private static void idleCompat() {
    if (linearOpMode != null) {
      linearOpMode.idle();
    }
  }

  /** Simple PIDF on a scalar error (inches or degrees). */
  private static final class Pidf {
    final double p;
    final double i;
    final double d;
    final double f;
    double integral;
    double lastErr;
    boolean hasLast;
    final ElapsedTime dtTimer = new ElapsedTime();

    Pidf(double p, double i, double d, double f) {
      this.p = p;
      this.i = i;
      this.d = d;
      this.f = f;
    }

    void reset() {
      integral = 0;
      lastErr = 0;
      hasLast = false;
      dtTimer.reset();
    }

    double update(double err, double maxAbs) {
      double dt = dtTimer.seconds();
      if (dt < 1e-3) {
        dt = 1e-3;
      }
      dtTimer.reset();
      integral += err * dt;
      if (i > 1e-9) {
        double iLimit = maxAbs / i;
        integral = Range.clip(integral, -iLimit, iLimit);
      } else {
        integral = 0;
      }
      double deriv = hasLast ? (err - lastErr) / dt : 0;
      lastErr = err;
      hasLast = true;
      double ff = (Math.abs(err) < 1e-6) ? 0 : f * Math.signum(err);
      return p * err + i * integral + d * deriv + ff;
    }
  }
}
