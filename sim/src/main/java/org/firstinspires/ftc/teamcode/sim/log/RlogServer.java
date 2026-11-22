package org.firstinspires.ftc.teamcode.sim.log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;

/**
 * AdvantageScope'a canlı veri gönderen TCP sunucusu (RLOG, varsayılan port 5800).
 * Davranış AdvantageKit'in RLOGServer'ı ile aynıdır: her mesajın önünde 4 baytlık uzunluk,
 * 20 ms'de bir yayın, istemciden 3 sn kalp atışı gelmezse bağlantı kapatılır.
 */
class RlogServer {
    private static final double HEARTBEAT_TIMEOUT_SECS = 3.0;

    private final int port;
    private final SimLog log;
    private final ArrayBlockingQueue<byte[]> queue = new ArrayBlockingQueue<>(500);
    private final List<Socket> sockets = new ArrayList<>();
    private final List<Double> lastHeartbeats = new ArrayList<>();
    private final Object socketsLock = new Object();
    private volatile boolean running = false;
    private ServerSocket server;

    RlogServer(int port, SimLog log) {
        this.port = port;
        this.log = log;
    }

    void start() {
        try {
            server = new ServerSocket(port);
        } catch (IOException e) {
            System.out.println("[Sim] RLOG sunucusu " + port + " portunda acilamadi: " + e.getMessage());
            return;
        }
        running = true;

        Thread accept = new Thread(this::acceptLoop, "Sim_RLOGAccept");
        accept.setDaemon(true);
        accept.start();

        Thread broadcast = new Thread(this::broadcastLoop, "Sim_RLOGBroadcast");
        broadcast.setDaemon(true);
        broadcast.start();

        System.out.println("[Sim] AdvantageScope icin RLOG sunucusu acildi: localhost:" + port);
    }

    void broadcast(byte[] cycle) {
        if (!running) return;
        // Yayın geride kalırsa bu döngüyü atla (AdvantageKit de böyle yapar)
        queue.offer(frame(cycle));
    }

    void stop() {
        running = false;
        try {
            if (server != null) server.close();
        } catch (IOException ignored) {
        }
        synchronized (socketsLock) {
            for (Socket s : sockets) {
                try {
                    s.close();
                } catch (IOException ignored) {
                }
            }
            sockets.clear();
            lastHeartbeats.clear();
        }
    }

    private static byte[] frame(byte[] data) {
        return ByteBuffer.allocate(4 + data.length).putInt(data.length).put(data).array();
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    private void acceptLoop() {
        while (running) {
            try {
                Socket socket = server.accept();
                socket.setTcpNoDelay(true);
                // Bekleyen döngüleri temizlemeden önce yeni istemciye mevcut durumu gönder
                synchronized (socketsLock) {
                    socket.getOutputStream().write(frame(log.newcomerData()));
                    sockets.add(socket);
                    lastHeartbeats.add(now());
                }
                System.out.println("[Sim] AdvantageScope baglandi: " + socket.getInetAddress().getHostAddress());
            } catch (IOException e) {
                if (running) System.out.println("[Sim] RLOG baglanti hatasi: " + e.getMessage());
            }
        }
    }

    private void broadcastLoop() {
        List<byte[]> pending = new ArrayList<>();
        while (running) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return;
            }
            pending.clear();
            queue.drainTo(pending);

            synchronized (socketsLock) {
                for (int i = sockets.size() - 1; i >= 0; i--) {
                    Socket socket = sockets.get(i);
                    try {
                        InputStream in = socket.getInputStream();
                        if (in.available() > 0) {
                            in.skip(in.available());
                            lastHeartbeats.set(i, now());
                        }
                        if (now() - lastHeartbeats.get(i) > HEARTBEAT_TIMEOUT_SECS) {
                            throw new IOException("zaman aşımı");
                        }
                        OutputStream out = socket.getOutputStream();
                        out.write(new byte[4]); // canlı tutma mesajı (uzunluk 0)
                        for (byte[] data : pending) out.write(data);
                        out.flush();
                    } catch (IOException e) {
                        try {
                            socket.close();
                        } catch (IOException ignored) {
                        }
                        sockets.remove(i);
                        lastHeartbeats.remove(i);
                        System.out.println("[Sim] AdvantageScope baglantisi kapandi (" + e.getMessage() + ")");
                    }
                }
            }
        }
    }
}
