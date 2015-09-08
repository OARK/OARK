#!/usr/bin/env python

"""A ROS node to coerce gstreamer into streaming to a desired 
IP address over UDP.
"""

import rospy
import sys
import argparse
import yaml
import re
import threading

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
GObject.threads_init()

from std_srvs.srv import Empty, EmptyResponse
from emumini2.srv import StartStream, StartStreamResponse

__author__    = 'Mike Aldred, Tim Peskett'
__copyright__ = 'Copyright 2016, OARK'
__credits__   = ['Tim Peskett', 'Mike Aldred']



class VideoStream(object):
    """Represents a video stream from a source to a address and port
    on a target machine.
    """

    #Match variables inside of double curly braces. e.g {{ var_name }}
    variable_re = r'^\s*\{\{\s*(\w+)\s*\}\}\s*$'

    def __init__(self, elements, **kwargs):
        """Initialises the video stream. Takes a list of dictionaries that specify
        the various configuration parameters for a GST element. The list of dictionaries
        has the following format:
        
        [
            {
                element: [
                            { 
                              name: factory_name,
                              properties: {
                                             prop1: val1
                                             prop2: val2
                                             ...
                                             }
                            },
                            { 
                              name: factory_name2,
                              ...
                            },
                            ...
                         ]
            },
            {
                element: [
                            {
                                ...
                            },
                            ...
                         ]
            },
        ]

        There are obviously some redunancies and annoyances with this format, but
        some of this is necessary to facilitate its writing in YAML.

        The elements of the inner dictionaries are tried in order until one of
        them succeeds. The element that succeeds will be the element added
        to the pipeline. The elements must exist in the order in which they
        will be added to the pipeline.

        Additionally, a variable value is denoted by double curly-braces.
        For example: {{ var_name }}. On processing, this variable name will
        be replaced by whatever value is passed in.

        Finally, any property named caps will have its value first passed to
        Gst.Caps.from_string() before being used as a property.

        See the documentation for _make_gst_element for more information.
        """

        self._pipeline = Gst.Pipeline()

        #Make each GST element based on its specification from the config
        gst_elements = map(lambda e: self._make_gst_element(e['element'], kwargs),
                           elements)

        #Create the pipeline using the GST elements
        for gst_element in gst_elements:
            self._pipeline.add(gst_element)

        #Link elements together
        reduce(VideoStream.link, gst_elements)



    @staticmethod
    def link(e1, e2):
        """Link two elements. Only links forwards, not backwards.
        This is a helper method for reduce
        """
        e1.link(e2)
        return e2


    def _make_gst_element(self, element_list, replacements):
        """Makes a GST element based on a dictionary.
        element_list should be a list of dictionaries, where each dictionary
        specifies the attributes of an element. *The first dictionary that
        specifies a valid, successfully created element will determine the
        element that is returned*. This is done to allow 'fallbacks' in case
        a required element cannot be created but there is another element that
        can take its place.

        The format of each element dictionary should be as follows:
        {
            name: factory_name,
            properties: {
                            prop1: value1,
                            prop2: value2,
                            ...
                        }
        }
        properties is an optional field.

        replacements is a dictionary of variable substitutions to make.
        """
        def substitute_var(string, sym_table):
            """A helper function to substitute the variable inside of sym_table into
            string if applicable. Returns string if string is not a variable.
            This code needs refactoring.
            """
            m = re.search(VideoStream.variable_re, string)
            if m is not None:
                print 'Made substitution of ', m.group(1), ' with ', sym_table[m.group(1)]
                string = sym_table[m.group(1)]
            return string

        try:
            #Loop through all alternative elements
            for element in element_list:
                #Resolve variable if necessary
                if isinstance(element['name'], basestring):
                    element['name'] = substitute_var(element['name'], replacements)
                gst_element = Gst.ElementFactory.make(element['name'], None)
                print 'Creating element ', element['name']

                if gst_element is not None:
                    print 'Successful'
                    #Loop through properties on the successful element
                    if 'properties' in element:
                        for prop_name, prop in element['properties'].iteritems():
                            #Resolve variable if necessary
                            if isinstance(prop, basestring):
                                prop = substitute_var(prop, replacements)

                            #Test whether prop is special capabilities property
                            if prop_name == 'caps':
                                gst_element.set_property(prop_name, 
                                                         Gst.Caps.from_string(prop))
                            else:
                                gst_element.set_property(prop_name, prop)
                    
                    #Element successfully created! No need to continue
                    return gst_element
        except KeyError, ke:
            raise StreamException('Bad variable substitution: ' + str(ke))

        #No successful element creation. Raise exception
        raise StreamException('Element creation failed!')



    def stream(self):
        """Begin streaming using the pipeline that we have constructed
        """
        self._pipeline.set_state(Gst.State.PLAYING)


    def shutdown(self):
        """Shutdown the stream as safely as possible.
        """
        self._pipeline.set_state(Gst.State.NULL)



class StreamNode(object):
    def __init__(self, node_namespace, config_filename, video_src, port):
        """Initialise the node so that it is ready to stream when called
        upon.
        Params:
            node_namespace  - The namespace that this node's services/topics
                              should be created under.
            config_filename - The filename of the file containing the pipeline
                              config to be used when constructing the gstreamer
                              pipeline.
            video_src       - The name of the video source to stream from. This
                              will usually be v4l2src.
            port            - The port to stream the data on.
        """
        rospy.loginfo("Init StreamNode video.")
        self._video_src = video_src
        self._port = port

        #Read the pipeline configuration from a file
        element_file = open(config_filename, 'r')
        self._elements = yaml.load(element_file.read())
        element_file.close()

        #Initialise gstreamer
        Gst.init(None)
        self._streamer = None

        #Create ROS services
        self._stream_mutex = threading.Lock()
        rospy.Service('/%s/start_stream'%(node_namespace,), StartStream, self.start_stream)
        rospy.Service('/%s/stop_stream'%(node_namespace,), Empty, self.stop_stream)


    def start_stream(self, req):
        """A ROS service to start streaming video towards a certain address.
        The address supplied should be a simple IPv4 address in the form
        of a string. This string is supplied directly to gstreamer's udpsink.
        """
        with self._stream_mutex:
            if self._streamer is not None:
                #Shut down old stream
                self._streamer.shutdown()

            success = True
            try:
                #initialise and start the streamer
                self._streamer = VideoStream(self._elements,
                                             source=self._video_src,
                                             address=req.address,
                                             port=self._port)
                self._streamer.stream()
                rospy.loginfo('Video streaming to ' + str(req.address))
            except StreamException, se:
                rospy.logerror('Could not start video stream: ' + str(se))
                self._streamer = None
                success = False

            #Return a C-style error code.
            return StartStreamResponse(1 if success else 0)


    def stop_stream(self, req):
        """A ROS service to stop streaming video.
        """
        with self._stream_mutex:
            if self._streamer is not None:
                self._streamer.shutdown()
                self._streamer = None

            rospy.loginfo('Stopping video stream...')

            return EmptyResponse()



class StreamException(Exception):
    pass


if __name__ == '__main__':
    """The entry point. Parse arguments. Setup ROS service.
    """

    #Initialise gstreamer

    parser = argparse.ArgumentParser(description='Start video streaming ROS node')
    parser.add_argument('element_file',
                        help='A file from which to load the gstreamer elements')
    parser.add_argument('video_src',
                        default='videotestsrc',
                        help='A string representing the source from which to stream video')
    parser.add_argument('port',
                        type=int,
                        default=5000,
                        help='The port to stream video on')
    parser.add_argument('--address',
                        help='The initial address to stream video to')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    node_name = 'em2_vid_node'
    rospy.init_node(node_name)
    try:
        sn = StreamNode(node_name, args.element_file, args.video_src, args.port)
        if args.address is not None:
            sn.start_stream(address)
        rospy.spin()
    except Exception, e:
        rospy.logerr('Error occurred: ' + str(e))
