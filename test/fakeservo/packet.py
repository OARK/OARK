#!/usr/bin/env python
#
# Contains a class representing an instruction packet and a 
# class representing a status packet for the AX12.
#
# See http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
#
# Written by Tim Peskett


class Packet(object):
    """Provides a base class for the other two packet types as they are
    very similar. An object of this class should *not* be instantiated.
    A subclass need only implement the _get_fields method as specified
    to properly utilise the inheritance.
    """
    def _get_fields(self):
        """This, and __init__ are the only methods that must be
        implemented in the base class. This should return all of
        the fields that make up the packet, except for the leading
        two 0xFF bytes and the checksum. The params should *not* be
        in their own list. All fields should be in the same order
        that they appear in the packet.
        """
        raise NotImplementedError('This is an abstract base class!')


    def calc_checksum(self):
        """Performs the AX12 checksum using the whatever fields
        are supplied. The checksum is very similar between packets,
        it just includes different fields.
        """

        return (~sum(self._get_fields()) % 0xFF)


    #Creates a list-of-bytes representation of the packet
    def serialize(self):
        """ Creates a list-of-bytes representation of a packet from a packet
        object. Adds the two 0xFF bytes at the beginning to ensure AX12
        compliance.
        """
        packet = [0xFF, 0xFF] + self._get_fields()
        packet.append(self.checksum)

        return packet




class InstructionPacket(Packet):
    """
    Represents a standard AX12 instruction packet. The format
    of the packet is as follows
        
            Byte    Value           Meaning
            ----    -----           -------
               0    0xFF            Start byte
               1    0xFF            Second start byte
               2    0x00-0xFE       The ID of the destination dynamixel. 
                                    0xFE is broadcast ID.
               3    0x02-0xFF       The length of the instruction packet.
                                    Set to number of parameters + 2.
               4    0x00-0x06,0x83  The instruction to perform.
             ...    0x00-0xFF       A parameter for the instruction.
            LAST    0x00-0xFF       The checksum.

    See http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
    for more details."""

    def __init__(self, dyn_id, instruction, param_list, length=None, checksum=None):
        self.dyn_id = dyn_id
        self.instruction = instruction
        self.params = param_list

        if length is not None:
            self.length = length
        else:
            self.length = len(params) + 2

        if checksum is not None:
            self.checksum = checksum
        else:
            self.checksum = self.calc_checksum()
            

    def _get_fields(self):
        return [self.dyn_id, self.length, self.instruction] + self.params


    @staticmethod
    def deserialize(packet):
        """Turn a list of bytes into a InstPacket. Assumes that the packet
        starts with 0xFF as dynamixel packets do.
        """
        #Remove first two 0xFF bytes
        packet = packet[2:]
        return InstPacket(packet[0], packet[2],
                          packet[3:-1], length=packet[1],
                          checksum=packet[-1])



class StatusPacket(Packet):
    """Represents a standard AX12 static packet. The format
    of the packet is as follows
        
            Byte    Value           Meaning
            ----    -----           -------
               0    0xFF            Start byte
               1    0xFF            Second start byte
               2    0x00-0xFE       The ID of the destination dynamixel. 
                                    0xFE is broadcast ID.
               3    0x02-0xFF       The length of the instruction packet.
                                    Set to number of parameters + 2.
               4    0x00-0x7F       The error code to return.
             ...    0x00-0xFF       A parameter for the instruction.
            LAST    0x00-0xFF       The checksum.

    See http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
    for more details."""


    def __init__(self, dyn_id, error, param_list, length=None, checksum=None):
        self.dyn_id = dyn_id
        self.error = error
        self.params = param_list

        if length is not None:
            self.length = length
        else:
            self.length = len(params) + 2

        if checksum is not None:
            self.checksum = checksum
        else:
            self.checksum = self.calc_checksum()
            

    def _get_fields(self):
        return [self.dyn_id, self.length, self.error] + self.params


    @staticmethod
    def deserialize(packet):
        """Turn a list of bytes into a InstPacket. Assumes that the packet
        starts with 0xFF as dynamixel packets do.
        """
        #Remove first two 0xFF bytes
        packet = packet[2:]
        return StatusPacket(packet[0], packet[2],
                            packet[3:-1], length=packet[1],
                            checksum=packet[-1])
