#!/usr/bin/env python

# Python module for handling the video stream from a robot. It's
# possible for this is be run either on a development machine, or a
# Pi.

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
GObject.threads_init()
Gst.init(None)

import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('oark.sensor_video')

import os

def _test_device_available(path):
    if os.path.isfile(path) and os.access(path, os.R_OK):
        return True
    else:
        logger.debug('%s not available', path)

class VideoSensor:
    video_src = None
    video_encoder = None
    rtp_payload = None
    video_src_caps_filter = None
    video_enc_caps_filter = None
    udp_sink = None
    _pipeline = None
    mainloop = None

    def __init__(self, source, target_ip):
        self.mainloop = GObject.MainLoop()
        self.setup_source(source)
        self._pipeline = Gst.Pipeline()

        self.setup_h264_parser()
        self.setup_encoding(1200000)
        self.setup_rtppayload()
        self.setup_video_src_caps_filter()
        self.setup_video_enc_caps_filter()
        self.setup_udp_target('192.168.9.33', 5000)
        self.setup_pipeline()
        self.link_video()

    # Gets the video source
    def setup_source(self, source_string):
        self.video_src = Gst.ElementFactory.make(source_string, 'video_src')

    def setup_h264_parser(self):
        self.h264_parser = Gst.ElementFactory.make('h264parse', 'h264parse')

    def setup_encoding(self, bitrate):
        # Check for the Pi hardware encoder first, otherwise check for
        # a software encoder
        self.video_encoder = Gst.ElementFactory.make('omxh264enc', 'video_enc')
        if self.video_encoder != None:
            self.video_encoder.set_property('control-rate', 'variable')
            self.video_encoder.set_property('target-bitrate', bitrate)
        else:
            self.video_encoder = Gst.ElementFactory.make('x264enc', 'video_enc')
            self.video_encoder.set_property('sliced-threads', True)
            self.video_encoder.set_property('tune', 0x00000004)

    # Check for RTP payload module
    def setup_rtppayload(self):
        self.rtp_payload = Gst.ElementFactory.make('rtph264pay', 'RTPH264Pay')
        self.rtp_payload.set_property('pt', 96)
        self.rtp_payload.set_property('config-interval', 4)

    def setup_video_src_caps_filter(self):
        self.video_src_caps_filter = Gst.ElementFactory.make('capsfilter', 'src_caps')
        self.video_src_caps_filter.set_property('caps', Gst.Caps.from_string('video/x-raw, width=640, height=480,framerate=25/1,color-rate=90000'))

    def setup_video_enc_caps_filter(self):
        self.video_enc_caps_filter = Gst.ElementFactory.make("capsfilter", 'dst_caps')
        self.video_enc_caps_filter.set_property('caps', Gst.Caps.from_string('video/x-h264'))

    def setup_udp_target(self, address, port):
        self.udp_sink = Gst.ElementFactory.make('udpsink', 'udpsink')
        self.udp_sink.set_property('host', address)
        self.udp_sink.set_property('port', port)

    def setup_pipeline(self):
        for element in [self.video_src, self.video_src_caps_filter,
                        self.video_encoder, self.video_enc_caps_filter,
                        self.h264_parser, self.rtp_payload, self.udp_sink]:
            logger.debug("Adding %s to the pipeline" % element)
            self._pipeline.add(element)

    def link_video(self):
        self.video_src.link(self.video_src_caps_filter)
        self.video_src_caps_filter.link(self.video_encoder)
        self.video_encoder.link(self.video_enc_caps_filter)
        self.video_enc_caps_filter.link(self.h264_parser)
        self.h264_parser.link(self.rtp_payload)
        self.rtp_payload.link(self.udp_sink)

    def start_video(self):
        self._pipeline.set_state(Gst.State.PLAYING)

    def run(self):
        self.start_video()
        self.mainloop.run()

if __name__ == "__main__":
    app = VideoSensor("videotestsrc", "192.168.9.33")
    app.run()
