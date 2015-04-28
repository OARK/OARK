package org.intelligentrobots.oarkcontroller.streams;

/*
 * This class handled the encapsulation of H264 frames in RTP packets.
 * This follows the RFC 6184 spec: http://tools.ietf.org/html/rfc6184.
 *
 * This class will take the payload of a RTP packet, and determine which of the
 * three payload structures it is.
 */

import java.util.Arrays;

import android.util.Log;

public class RtpH264 {
    // Contains the RAW mPayload of the RTP packet.
    private byte[] mPayload;

    private int mNalRefIdc;
    private int mNalUnitType;

    private int mNalSize;

    private int mHeaderLen;

    private int mOutSize;
    private byte[] mOutBuffer;

    private final static String TAG = "RtpH264: ";

    // Wait for the start packet before we process.
    private boolean mWaitStart = true;

    private boolean mBufferReady = false;

    private final static byte[] SYNC_BYTES = new byte[] {
            (byte) 0, (byte) 0, (byte) 0, (byte) 1};

    // Flags for start, end bits for fragmentation.
    private boolean mStartBit, mEndBit;

    public byte[] getPayload() {
        mBufferReady = false;
        return mPayload;
    }

    public String getPacketType() {
        String result;

        result = "NAL Ref: " + mNalRefIdc +
            "\nNAL Unit Type: " + mNalUnitType +
            "\nStartBit: " + mStartBit +
            "\nEndBit: " + mEndBit +
            "\nOutSize: " + mOutSize +
            "\nReady: " + mBufferReady;

        return result;
    }

    public boolean isReady() {
        return mBufferReady;
    }

    public byte[] getOutputBuffer() {
        return mOutBuffer.clone();
    }

    public void setPayload(byte[] inpayload, int len) {
        mPayload = new byte[len];
        for (int i = 0; i < len; i++) {
            mPayload[i] = inpayload[i];
        }
    }

    public void parsePacket() {
        mNalRefIdc = (mPayload[0] & 0x60) >> 5;
        mNalUnitType = mPayload[0] & 0x1f;

        // Have at least one byte of header when we have a payload type.
        mHeaderLen = 1;

        Log.d(TAG, "parsingPacket");

        switch (mNalUnitType) {
        case 0:
        case 30:
        case 31:
            Log.d(TAG, "undefined type");
            // Undefined
            break;
        case 25:
            Log.d(TAG, "STAP-B");
            /* STAP-B Single time aggregation packet. */
            /* 2 byte extra header for DON. */
            mHeaderLen += 2;
            // Fall through
        case 24:
            Log.d(TAG, "STAP-A");
            /* STAP-A Single time aggregation packet. */
            // Strip any headers
            mPayload = Arrays.copyOfRange(mPayload, mHeaderLen, mPayload.length - mHeaderLen);

            mWaitStart = false;

            while (mPayload.length > 2) {
                /*                      1
                 *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
                 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
                 * |         NALU Size             |
                 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
                 */
                mNalSize = (mPayload[0] << 8) | mPayload[1];

                /* Strip Nal size */
                mPayload = Arrays.copyOfRange(mPayload, 2, mPayload.length - 2);

                if (mNalSize > mPayload.length) {
                    mNalSize = mPayload.length;
                }

                mOutSize = mNalSize + SYNC_BYTES.length;
                mOutBuffer = new byte[mOutSize];

                System.arraycopy(SYNC_BYTES, 0, mOutBuffer, 0, SYNC_BYTES.length);
                System.arraycopy(mPayload, 0, mOutBuffer, SYNC_BYTES.length, mPayload.length);
            }
            break;
        case 26:
            /* MTAP Multi-time aggregation packet. */
            mHeaderLen = 5;
        case 27:
            /* MTAP Multi-time aggregation packet. */
            mHeaderLen = 6;
            break;
        case 28:
            Log.d(TAG, "Case 28");
        case 29:
            Log.d(TAG, "FU packets");
            /*
             * FU-A Fragmentation unit
             * FU-B Fragmentation unit
             */

            /* +---------------+
             * |0|1|2|3|4|5|6|7|
             * +-+-+-+-+-+-+-+-+
             * |S|E|R|  Type   |
             * +---------------+
             *
             * R is reserved and always 0
             */

            mStartBit = (mPayload[1] & 0x80) == 0x80;
            mEndBit = (mPayload[1] & 0x40) == 0x40;

            if (mWaitStart && !mStartBit) {
                Log.d(TAG, "Waiting for start, no start bit.");
                return;
            }

            if (mStartBit) {
                // Reconstruct NAL header
                byte nalHeader = (byte) ((mPayload[0] & 0xe0) | (mPayload[1] & 0x1f));

                mWaitStart = false;

                /* Strip type header, keep FU header. */
                mPayload = Arrays.copyOfRange(mPayload, 1, mPayload.length - 1);

                mNalSize = mPayload.length;

                mOutSize = mNalSize + SYNC_BYTES.length;

                mOutBuffer = new byte[mOutSize];

                System.arraycopy(SYNC_BYTES, 0, mOutBuffer, 0, SYNC_BYTES.length);
                System.arraycopy(mPayload, 0, mOutBuffer, SYNC_BYTES.length, mPayload.length);
                mOutBuffer[0] = nalHeader;

                Log.d(TAG, "Queuing " + mOutSize + " bytes");
            } else {
                /* Strip off FU indicator and FU header. */

                mPayload = Arrays.copyOfRange(mPayload, 2, mPayload.length - 2);
                mOutSize = mPayload.length + mOutBuffer.length;

                byte[] tempBuffer = new byte[mOutSize];

                System.arraycopy(mOutBuffer, 0, tempBuffer, 0, mOutBuffer.length);
                System.arraycopy(mPayload, 0, tempBuffer, mOutBuffer.length, mPayload.length);

                Log.d(TAG, "Queuing " + mOutSize + " bytes");
                mOutBuffer = tempBuffer;

            }

            if (mEndBit) {
                Log.d(TAG, "Output " + mOutSize + " bytes");
                // Buffer should be ready
                mBufferReady = true;
            }
            break;
        default:
            Log.d(TAG, "Default");
            mWaitStart = false;
            // Single NAL unit
            mNalSize = mPayload.length;

            mOutSize = mNalSize + SYNC_BYTES.length;

            mOutBuffer = new byte[mOutSize];

            System.arraycopy(SYNC_BYTES, 0, mOutBuffer, 0, SYNC_BYTES.length);
            System.arraycopy(mPayload, 0, mOutBuffer, SYNC_BYTES.length, mPayload.length);
            mBufferReady = true;
        }
    }
}
