package sep_17.emumini2;

import android.util.Log;

import java.io.IOException;
import java.io.DataOutputStream;
import java.net.InetAddress;
import java.net.Socket;

/**
 * Created by 16243969 on 28/04/15.
 *
 * TODO: Add better inception handling
 */
public class Talker {
    public final int DEFAULT_PORT = 1717;

    private boolean init = false;

    private String address;
    private int port;

    private Socket sock;
    private DataOutputStream dos;

    public Talker(String address) throws IOException {
        this.address = address;
        port = DEFAULT_PORT;

        TalkerInit ti = new TalkerInit();
        new Thread(ti).start();
    }

    public Talker(String address, int port) throws IOException {
        this.address = address;
        this.port = port;

        TalkerInit ti = new TalkerInit();
        new Thread(ti).start();
    }

    /* This inner class is necessary so that we don't try to get the
     * socket inside of the UI thread.
     * Right now this code is very susceptible to race conditions.
     */
    class TalkerInit implements Runnable {
        public TalkerInit() {
        }

        public void run() {
            try {
                sock = new Socket(InetAddress.getByName(address), port);

                dos = new DataOutputStream(sock.getOutputStream());

                init = true;
            }
            catch(IOException e) {
                Log.e("EMUMINI2", "No dice");
            }
        }
    }

    public boolean isInit() {
        return init;
    }

    public void close() throws IOException {
        sock.close();
    }

    public synchronized void send(IMsg msg) throws IOException {
        dos.writeByte(msg.getSize());
        msg.serialise(dos);
        dos.flush();
    }
}
