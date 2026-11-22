package org.firstinspires.ftc.teamcode.sim.log;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * AdvantageScope'un okuyabildiği RLOG (R2) formatında veri kaydı.
 * - Canlı: RlogServer üzerinden (AdvantageScope > File > Connect to Simulator > RLOG Server, port 5800)
 * - Dosya: sim/logs/*.rlog (AdvantageScope'ta File > Open Log ile açılır)
 *
 * Format AdvantageKit'in RLOGEncoder'ı ile birebir aynıdır
 * (https://github.com/Mechanical-Advantage/AdvantageKit, BSD lisansı).
 *
 * Kullanım her döngüde: beginCycle(t) -> putXxx(...) -> endCycle()
 */
public class SimLog implements AutoCloseable {
    public static final byte LOG_REVISION = 2;

    private static final class Value {
        final String type;
        final byte[] data;

        Value(String type, byte[] data) {
            this.type = type;
            this.data = data;
        }

        boolean sameAs(Value o) {
            return o != null && type.equals(o.type) && Arrays.equals(data, o.data);
        }
    }

    // Değerler döngüler arasında korunur (son değer yeni bağlanan istemciye gönderilir)
    private final LinkedHashMap<String, Value> table = new LinkedHashMap<>();
    private final Map<String, Value> lastSent = new HashMap<>();
    private final LinkedHashMap<String, Short> keyIds = new LinkedHashMap<>();
    private short nextKeyId = 0;
    private double timestamp = 0.0;

    private final Object lock = new Object();
    private RlogServer server;
    private OutputStream file;
    private long lastFlush = 0;

    public SimLog() {
        // WPILib struct şemaları: AdvantageScope Pose2d'yi bunlarla çözer
        putSchema("Translation2d", "double x;double y");
        putSchema("Rotation2d", "double value");
        putSchema("Pose2d", "Translation2d translation;Rotation2d rotation");
    }

    /** Canlı yayını başlatır. */
    public void startServer(int port) {
        server = new RlogServer(port, this);
        server.start();
    }

    /** sim/logs klasörüne .rlog dosyası yazmaya başlar. */
    public File startFile(File dir) throws IOException {
        if (!dir.exists() && !dir.mkdirs()) throw new IOException("Klasör oluşturulamadı: " + dir);
        String name = "sim_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()) + ".rlog";
        File f = new File(dir, name);
        file = new BufferedOutputStream(new FileOutputStream(f));
        file.write(LOG_REVISION);
        return f;
    }

    // ------------------------------------------------------------------ döngü

    public void beginCycle(double timestampSeconds) {
        this.timestamp = timestampSeconds;
    }

    public void endCycle() {
        byte[] cycle;
        synchronized (lock) {
            ByteArrayBuilder out = new ByteArrayBuilder();
            out.put((byte) 0).putDouble(timestamp);
            for (Map.Entry<String, Value> e : table.entrySet()) {
                Value v = e.getValue();
                if (v.sameAs(lastSent.get(e.getKey()))) continue;
                Short id = keyIds.get(e.getKey());
                if (id == null) {
                    id = nextKeyId++;
                    keyIds.put(e.getKey(), id);
                    encodeKey(out, id, e.getKey(), v.type);
                }
                encodeValue(out, id, v);
                lastSent.put(e.getKey(), v);
            }
            cycle = out.toArray();
        }

        if (server != null) server.broadcast(cycle);
        if (file != null) {
            try {
                file.write(cycle);
                long now = System.nanoTime();
                if (now - lastFlush > 1_000_000_000L) {
                    file.flush();
                    lastFlush = now;
                }
            } catch (IOException e) {
                System.out.println("[Sim] Log dosyasi yazilamadi: " + e);
                file = null;
            }
        }
    }

    /** Yeni bağlanan AdvantageScope için: revizyon + tüm anahtarlar + tüm son değerler. */
    byte[] newcomerData() {
        synchronized (lock) {
            ByteArrayBuilder out = new ByteArrayBuilder();
            out.put(LOG_REVISION);
            out.put((byte) 0).putDouble(timestamp);
            for (Map.Entry<String, Short> e : keyIds.entrySet()) {
                encodeKey(out, e.getValue(), e.getKey(), lastSent.get(e.getKey()).type);
            }
            for (Map.Entry<String, Short> e : keyIds.entrySet()) {
                encodeValue(out, e.getValue(), lastSent.get(e.getKey()));
            }
            return out.toArray();
        }
    }

    // ------------------------------------------------------------------ değer yazma

    public void putDouble(String key, double value) {
        put(key, "double", ByteBuffer.allocate(8).putDouble(value).array());
    }

    public void putBoolean(String key, boolean value) {
        put(key, "boolean", new byte[]{(byte) (value ? 1 : 0)});
    }

    public void putString(String key, String value) {
        put(key, "string", value.getBytes(StandardCharsets.UTF_8));
    }

    public void putDoubleArray(String key, double[] values) {
        ByteBuffer b = ByteBuffer.allocate(values.length * 8);
        for (double v : values) b.putDouble(v);
        put(key, "double[]", b.array());
    }

    /** WPILib Pose2d: metre ve radyan, AdvantageScope saha koordinatlarında. */
    public void putPose2d(String key, double xMeters, double yMeters, double headingRad) {
        put(key, "struct:Pose2d", poseBytes(new double[][]{{xMeters, yMeters, headingRad}}));
    }

    /** Pose2d dizisi (ör. path'in noktaları -> AdvantageScope'ta çizgi olarak görünür). */
    public void putPose2dArray(String key, double[][] poses) {
        put(key, "struct:Pose2d[]", poseBytes(poses));
    }

    private static byte[] poseBytes(double[][] poses) {
        // Struct verileri little-endian'dır
        ByteBuffer b = ByteBuffer.allocate(poses.length * 24).order(ByteOrder.LITTLE_ENDIAN);
        for (double[] p : poses) {
            b.putDouble(p[0]).putDouble(p[1]).putDouble(p[2]);
        }
        return b.array();
    }

    private void putSchema(String name, String schema) {
        put("/.schema/struct:" + name, "structschema", schema.getBytes(StandardCharsets.UTF_8));
    }

    private void put(String key, String type, byte[] data) {
        if (!key.startsWith("/")) key = "/" + key;
        synchronized (lock) {
            table.put(key, new Value(type, data));
        }
    }

    // ------------------------------------------------------------------ kodlama

    private static void encodeKey(ByteArrayBuilder out, short id, String key, String type) {
        byte[] k = key.getBytes(StandardCharsets.UTF_8);
        byte[] t = type.getBytes(StandardCharsets.UTF_8);
        out.put((byte) 1).putShort(id).putShort((short) k.length).put(k).putShort((short) t.length).put(t);
    }

    private static void encodeValue(ByteArrayBuilder out, short id, Value v) {
        out.put((byte) 2).putShort(id).putShort((short) v.data.length).put(v.data);
    }

    @Override
    public void close() {
        if (server != null) server.stop();
        if (file != null) {
            try {
                file.flush();
                file.close();
            } catch (IOException ignored) {
            }
        }
    }

    /** Büyüyen big-endian bayt dizisi. */
    static final class ByteArrayBuilder {
        private ByteBuffer buf = ByteBuffer.allocate(4096);

        private void ensure(int n) {
            if (buf.remaining() < n) {
                ByteBuffer bigger = ByteBuffer.allocate(Math.max(buf.capacity() * 2, buf.position() + n));
                buf.flip();
                bigger.put(buf);
                buf = bigger;
            }
        }

        ByteArrayBuilder put(byte b) { ensure(1); buf.put(b); return this; }
        ByteArrayBuilder put(byte[] b) { ensure(b.length); buf.put(b); return this; }
        ByteArrayBuilder putShort(short s) { ensure(2); buf.putShort(s); return this; }
        ByteArrayBuilder putDouble(double d) { ensure(8); buf.putDouble(d); return this; }

        byte[] toArray() {
            return Arrays.copyOf(buf.array(), buf.position());
        }
    }
}
