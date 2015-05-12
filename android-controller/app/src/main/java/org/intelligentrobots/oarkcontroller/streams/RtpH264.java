package org.intelligentrobots.oarkcontroller.streams;

/*
 * This class handled the encapsulation of H264 frames in RTP packets.
 * This follows the RFC 6184 spec: http://tools.ietf.org/html/rfc6184.
 *
 * This class will take the payload of a RTP packet, and determine which of the
 * three payload structures it is.
 *
 * It only implements a limited subset of the spec, just enough to get
 * the specific payload from the OARK camera at time of writing.
 */

import java.io.ByteArrayOutputStream;

import android.util.Log;

import org.sipdroid.net.RtpPacket;

public class RtpH264 {
    /**
     * When we reconstruct NAL units from RTP packets, we may have to build
     * across packets, so this is a buffer that will persist across boundaries.
     */
    private ByteArrayOutputStream mOutputBuffer;

    /**
     * Prefix byte sequence for any NAL unit. This is part of the
     * Annex B spec for H264. This actually isn't in RFC 6184.
     * However, it seems some decoders require these to be present.
     *
     * TODO: Find out if there are hardware decoders that will crash
     * if they are present.
     */
    private final static byte[] SYNC_BYTES = {0, 0, 1};

    /**
     * Are we currently processing a fragmented NAL unit across packets.
     */
    private boolean mProcessingFragmentPacket = false;

    /**
     * Keep track of the last sequence number so we can ignore any repeated
     * packets, etc.
     */
    private long mLastSequenceNo = -1;

    /**
     * If we have incorrect data in the buffer, set this to true so it
     * can be discarded so we can start processing new data.
     */
    private boolean mDiscardBuffer;

    private String TAG = "NewRtp:";

    static public enum ProcessResult {
        BUFFER_PROCESSED_OK,
        OUTPUT_BUFFER_NOT_FILLED
    }

    /**
     * Allocates the buffer used for parsing the RTPH264 packets.
     */
    public RtpH264() {
        mOutputBuffer = new ByteArrayOutputStream();
        mProcessingFragmentPacket = false;
        mLastSequenceNo = -1;
    }

    /**
     * Extract the fragment of a NAL unit given a FU-A, FU-B RTP packet payload.
     *
     * @param inPayload the payload of the RTP packet.
     */
    private ProcessResult processFUPacket(byte[] inPayload) {
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

        /*
         * Second byte is the FU header.
         */


        byte nalHeader = (byte) ((inPayload[0] & 0xe0) | (inPayload[1] & 0x1f));

        boolean startBit = (inPayload[1] & 0x80) == 0x80;
        boolean endBit = (inPayload[1] & 0x40) == 0x40;

        mDiscardBuffer = false;

        boolean continueProcessing = true;

        ProcessResult result = ProcessResult.BUFFER_PROCESSED_OK;

        if (startBit) {
            /*
             * The start bit and end bit can't be in the same FU header.
             */
            if (endBit) {
                mDiscardBuffer = true;
                continueProcessing = false;
            } else {
                mProcessingFragmentPacket = true;
            }
        } else if (!mProcessingFragmentPacket) {
            mDiscardBuffer = true;
            continueProcessing = false;
        }

        if (continueProcessing) {

            if (startBit) {
                mOutputBuffer.write(SYNC_BYTES, 0, SYNC_BYTES.length);
                mOutputBuffer.write(nalHeader);
            }

            mOutputBuffer.write(inPayload, 2, inPayload.length - 2);

            if (endBit) {
                mProcessingFragmentPacket = false;
            } else {
                mProcessingFragmentPacket = true;
                result = ProcessResult.OUTPUT_BUFFER_NOT_FILLED;
            }
        }

        return result;
    }

    /**
     * Extract a single (complete) NAL unit from RTP payload.
     *
     * @param inNalUnitType unit type of NAL.
     * @param inPayload RTP payload.
     */
    private ProcessResult processSingleNALUnitPacket(
                                                     int inNalUnitType,
                                                     byte[] inPayload) {
        //
        mOutputBuffer.write(SYNC_BYTES, 0, SYNC_BYTES.length);
        mOutputBuffer.write(inPayload, 0, inPayload.length);

        return ProcessResult.BUFFER_PROCESSED_OK;
    }

    /**
     * Processes a buffer.
     *
     * @param inRtp Rtp packet in.
     */
    public ProcessResult doProcess(RtpPacket inRtp) {
        byte[] payload = inRtp.getPayload();

        ProcessResult result = ProcessResult.OUTPUT_BUFFER_NOT_FILLED;

        boolean continueProcessing = true;

        if ((mLastSequenceNo != -1) &&
            ((inRtp.getSequenceNumber() - mLastSequenceNo) != 1)) {
            /*
             * Even if (the new) sequence number is less than the last sequence
             * number, we have to use it because the received sequence numbers
             * may have wrapped around.
             */
            Log.d(TAG, "Missing sequence packet, clear and retry.");
            result = reset();

            if (result == ProcessResult.OUTPUT_BUFFER_NOT_FILLED) {
                mLastSequenceNo = inRtp.getSequenceNumber();
                continueProcessing = false;
            }
        }

        if (continueProcessing && payload.length > 0) {
            mLastSequenceNo = inRtp.getSequenceNumber();

            int nalUnitType = payload[0] & 0x1f;

            // Single NAL unit packet.
            if ((nalUnitType >= 1) && (nalUnitType <= 23)) {
                mProcessingFragmentPacket = false;

                result = processSingleNALUnitPacket(nalUnitType,
                                                    payload);
            } else if (nalUnitType == 28) {
                // FU-A Fragmentation unit (FU)
                result = processFUPacket(payload);

                if (mDiscardBuffer) {
                    mProcessingFragmentPacket = false;
                    Log.d(TAG, "Discarding buffer.");
                    mOutputBuffer.reset();
                }
            }
        }

        return result;
    }

    public byte[] getOutputBuffer() {
        return mOutputBuffer.toByteArray();
    }

    private ProcessResult reset() {
        mProcessingFragmentPacket = false;
        mOutputBuffer.reset();

        return ProcessResult.OUTPUT_BUFFER_NOT_FILLED;
    }

    public int currentLength() {
        return mOutputBuffer.size();
    }

    public boolean ready() {
        return !mProcessingFragmentPacket;
    }

    public void clear() { mOutputBuffer.reset(); }

    public void discardBuffer() { mDiscardBuffer = true; }
}
