package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.shared.AutoPaths;
import org.firstinspires.ftc.teamcode.shared.DriverInput;
import org.firstinspires.ftc.teamcode.shared.PedroTuning;
import org.firstinspires.ftc.teamcode.shared.TeleopDrive;
import org.firstinspires.ftc.teamcode.sim.log.SimLog;

import java.io.File;
import java.util.List;

/**
 * Robotsuz sürüş simülatörü.
 *
 * Gerçek Pedro Follower'ı, robottaki Foresight katsayılarıyla ve robottaki TeleopDrive
 * koduyla çalıştırır; sadece motorlar ve Pinpoint yerine fizik modeli kullanılır.
 * Sonuç AdvantageScope'a canlı gönderilir (File > Connect to Simulator > RLOG Server, port 5800) ve sim/logs/ altına kaydedilir.
 *
 * Kumanda (teleop kontrollerine ek olarak):
 *   A : robotu başlangıç noktasına koy ve örnek otonomu başlat
 *   B : otonomu iptal et (çubukları oynatmak da iptal eder)
 *   Y : robotu başlangıç noktasına ışınla
 *
 * Argümanlar:
 *   --auto        Kumanda olmadan otonomu sürekli tekrar çalıştır
 *   --seconds N   N saniye sonra kapan (test için)
 */
public class SimMain {
    private static final double INCH_TO_METER = 0.0254;

    public static void main(String[] args) throws Exception {
        boolean autoOnly = false;
        double runSeconds = Double.POSITIVE_INFINITY;
        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("--auto")) autoOnly = true;
            if (args[i].equals("--seconds") && i + 1 < args.length) runSeconds = Double.parseDouble(args[++i]);
        }

        // ---- Robot ----
        Pose start = AutoPaths.START;
        MecanumPhysics physics = new MecanumPhysics(start);
        SimDrivetrain drivetrain = new SimDrivetrain(physics);
        Follower follower = new Follower(
                new SimLocalizer(physics),
                drivetrain,
                new Foresight(PedroTuning.foresightConfig));
        follower.setPose(start);

        TeleopDrive teleop = new TeleopDrive();
        DriverInput input = new DriverInput();

        // ---- Kayıt ----
        SimLog log = new SimLog();
        log.startServer(SimConfig.RLOG_PORT);
        if (SimConfig.WRITE_LOG_FILE) {
            File f = log.startFile(new File("logs"));
            System.out.println("[Sim] Log dosyasi: " + f.getAbsolutePath());
        }
        List<Path> auto = AutoPaths.exampleAuto();
        log.putPose2dArray("Auto/FullRoute", sampleRoute(auto));

        // ---- Kumanda ----
        DesktopGamepad gamepad = autoOnly ? null : DesktopGamepad.tryCreate();
        if (!autoOnly && gamepad == null) {
            System.out.println("[Sim] Kumanda yok, --auto modunda devam ediliyor.");
            autoOnly = true;
        }
        printHelp(autoOnly);

        Runtime.getRuntime().addShutdownHook(new Thread(log::close));

        // ---- Döngü ----
        int pathIndex = -1; // -1 = teleop
        final long startNanos = System.nanoTime();
        long lastNanos = startNanos;
        long nextLoop = startNanos;
        boolean wasConnected = false;
        boolean hintShown = false;

        while (true) {
            long now = System.nanoTime();
            double t = (now - startNanos) / 1e9;
            double dt = Math.min(0.1, (now - lastNanos) / 1e9);
            lastNanos = now;
            if (t > runSeconds) break;

            boolean connected = gamepad != null && gamepad.read(input);
            if (connected && !wasConnected) System.out.println("[Sim] Surmeye hazirsin.");
            if (!connected && !autoOnly && !hintShown && t > 1.0) {
                System.out.println("[Sim] Kumanda takili degil. Takinca otomatik baglanir."
                        + " Kumandasiz otonom izlemek icin: --args=\"--auto\"");
                hintShown = true;
            }
            wasConnected = connected;

            // --- Otonom başlat / iptal ---
            boolean startAuto = input.aPressed() || (autoOnly && pathIndex < 0);
            if (startAuto) {
                physics.setPose(start);
                follower.setPose(start);
                pathIndex = 0;
                follower.follow(auto.get(0));
                System.out.println("[Sim] Otonom basladi.");
            }
            if (input.yPressed()) {
                physics.setPose(start);
                follower.setPose(start);
            }
            if (pathIndex >= 0 && !autoOnly && (input.bPressed() || input.sticksActive(0.2))) {
                pathIndex = -1;
                System.out.println("[Sim] Otonom iptal edildi, teleop.");
            }

            if (pathIndex >= 0) {
                if (!follower.isBusy()) {
                    pathIndex++;
                    if (pathIndex < auto.size()) {
                        follower.follow(auto.get(pathIndex));
                    } else {
                        System.out.printf("[Sim] Otonom bitti (%.1f s).%n", t);
                        pathIndex = -1; // --auto modunda bir sonraki döngüde yeniden başlar
                    }
                }
            } else if (!autoOnly) {
                teleop.update(follower, input);
            }

            // --- Pedro + fizik ---
            follower.update(dt);
            int steps = Math.max(1, (int) Math.round(dt / SimConfig.PHYSICS_STEP));
            for (int i = 0; i < steps; i++) physics.step(dt / steps);

            // --- AdvantageScope ---
            log.beginCycle(t);
            Pose pose = follower.pose();
            putFieldPose(log, "Drive/Pose", pose);
            log.putDoubleArray("Drive/PedroPose", new double[]{pose.x(), pose.y(), Math.toDegrees(pose.heading())});
            log.putDoubleArray("Drive/FieldVelocity", physics.fieldVelocity());
            log.putDoubleArray("Drive/WheelPowers", physics.wheelPowers());
            log.putDoubleArray("Drive/Command", new double[]{
                    drivetrain.lastPowers().forward(), drivetrain.lastPowers().strafe(), drivetrain.lastPowers().turn()});
            log.putString("Follower/Mode", follower.mode().name());
            log.putBoolean("Follower/Busy", follower.isBusy());
            log.putDouble("Auto/PathIndex", pathIndex);
            if (pathIndex >= 0 && follower.following()) {
                putFieldPose(log, "Follower/ClosestPose", follower.closestPose());
                log.putPose2dArray("Follower/CurrentPath", samplePath(follower.currentPath()));
                log.putDouble("Follower/Completion", follower.completion());
            }
            log.putBoolean("Teleop/FieldCentric", teleop.isFieldCentric());
            log.putBoolean("Sim/GamepadConnected", connected);
            log.putDoubleArray("Gamepad/Sticks", new double[]{
                    input.leftStickX, input.leftStickY, input.rightStickX, input.rightStickY});
            log.putDouble("Sim/LoopMs", dt * 1000.0);
            log.endCycle();

            input.endCycle();

            // --- Sabit döngü periyodu ---
            nextLoop += (long) (SimConfig.LOOP_PERIOD * 1e9);
            long sleep = nextLoop - System.nanoTime();
            if (sleep > 0) {
                Thread.sleep(sleep / 1_000_000L, (int) (sleep % 1_000_000L));
            } else {
                nextLoop = System.nanoTime();
            }
        }

        if (gamepad != null) gamepad.close();
        log.close();
        System.out.println("[Sim] Kapandi.");
    }

    // ---------------------------------------------------------------- yardımcılar

    /** Pedro (inç, köşe orijin) -> AdvantageScope FTC sahası (metre, merkez orijin). */
    static double[] toField(Pose p) {
        double x = (p.x() - SimConfig.FIELD_SIZE / 2) * INCH_TO_METER;
        double y = (p.y() - SimConfig.FIELD_SIZE / 2) * INCH_TO_METER;
        double r = Math.toRadians(SimConfig.PEDRO_TO_FIELD_ROTATION_DEG);
        double cos = Math.cos(r), sin = Math.sin(r);
        return new double[]{x * cos - y * sin, x * sin + y * cos, MecanumPhysics.normalize(p.heading() + r)};
    }

    private static void putFieldPose(SimLog log, String key, Pose p) {
        double[] f = toField(p);
        log.putPose2d(key, f[0], f[1], f[2]);
    }

    private static double[][] samplePath(Path path) {
        int n = 30;
        double[][] out = new double[n + 1][];
        for (int i = 0; i <= n; i++) out[i] = toField(path.get(i / (double) n));
        return out;
    }

    private static double[][] sampleRoute(List<Path> paths) {
        double[][][] parts = new double[paths.size()][][];
        int total = 0;
        for (int i = 0; i < paths.size(); i++) {
            parts[i] = samplePath(paths.get(i));
            total += parts[i].length;
        }
        double[][] out = new double[total][];
        int k = 0;
        for (double[][] part : parts) for (double[] p : part) out[k++] = p;
        return out;
    }

    private static void printHelp(boolean autoOnly) {
        System.out.println();
        System.out.println("==================== FTC 24230 SIMULATOR ====================");
        System.out.println(" AdvantageScope: File > Connect to Simulator > RLOG Server  (port "
                + SimConfig.RLOG_PORT + ")");
        if (autoOnly) {
            System.out.println(" Mod: OTONOM (kumanda yok). Ornek otonom surekli tekrarlanir.");
        } else {
            System.out.println(" Sol cubuk: sur | Sag cubuk: don | LB: yavas | Back: field-centric");
            System.out.println(" A: otonomu baslat | B: iptal | Y: baslangica isinlan");
        }
        System.out.println(" Kapatmak icin: Ctrl+C");
        System.out.println("=============================================================");
        System.out.println();
    }
}
