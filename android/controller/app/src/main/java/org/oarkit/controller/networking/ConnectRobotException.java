/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.networking;

/**
 * Exception for network connections to the robot.
 *
 * These are higher level exceptions, it is expected that the lower
 * level IO and network exceptions are caught and wrapped by this.
 */
public class ConnectRobotException extends RuntimeException {
    // The different errors we can get.
    public enum ErrorCode {
        CANT_CONNECT, CONNECTION_DROPPED, CORRUPT_DATA, UNKNOWN;
    }

    private ErrorCode mErrorCode;

    public ConnectRobotException(Exception e, ErrorCode inErrorCode) {
        super(e);
        setErrorCode(inErrorCode);
    }

    public void setErrorCode(ErrorCode inErrorCode) {
        mErrorCode = inErrorCode;
    }
}
