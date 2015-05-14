package sep_17.emumini2;

import java.io.DataOutputStream;
import java.io.IOException;

import sep_17.emumini2.IMsg;
/**
 * A message model class.
 * This class is highly coupled to the application at hand. It is intended
 * as a quick prototype class to facilitate testing.
 *
 * The message has the same format that is documented inside msg.py of the
 * four wheel wonder project.
 *
 * Notably, only signed types are sent over the network because that is
 * what java is good at.
 *
 * TODO: Better inception handling
 */
public class FourWWMsg implements IMsg {
    /* Current messages types */
    /* Still ill defined as this is prototype */
    public final static byte NOOP = 0;
    public final static byte PING = 1; //Unimplemented
    public final static byte STOP = 2;
    public final static byte LEFT_GO = 3; // Left wheels on
    public final static byte RIGHT_GO = 4; //Right wheels on
    public final static byte ALL_GO = 5; // All wheels on
    public final static byte ARM_GO = 6; // Set arm position
    public final static byte TURN_CCW = 7;
    public final static byte TURN_CW = 8;
    public final static byte WRIST_GO = 9;
    public final static byte HAND_GO = 10;

    private byte type;
    private byte id;
    private byte val;
    private int duration;

    public FourWWMsg() {
        type = NOOP;
        id = val = 0;
        duration = 0;
    }

    public FourWWMsg(byte type, byte id, byte val, int duration) {
        this.type = type;
        this.id = id;
        this.val = val;
        this.duration = duration;
    }

    @Override
    public void serialise(DataOutputStream dos) throws IOException {
        dos.writeByte(type);
        dos.writeByte(id);
        dos.writeByte(val);
        dos.writeInt(duration);
    }

    /* Returns the sum of the size (in bytes) of the member fields */
    @Override
    public int getSize() {
        /* Gross, but java */
        return 1 + 1 + 1 + 4;
    }

    public byte getType() {
        return type;
    }

    public void setType(byte type) {
        this.type = type;
    }

    public byte getId() {
        return id;
    }

    public void setId(byte id) {
        this.id = id;
    }

    public byte getVal() {
        return val;
    }

    public void setVal(byte val) {
        this.val = val;
    }

    public int getDuration() {
        return duration;
    }

    public void setDuration(int duration) {
        this.duration = duration;
    }
}
