package sep_17.fourwwcontrol;

import java.io.DataOutputStream;
import java.io.IOException;

/**
 * Simple interface for Msg classes to reduce coupling a
 * little bit.
 */
public interface IMsg {
    public int getSize();
    public void serialise(DataOutputStream dos) throws IOException;

}
