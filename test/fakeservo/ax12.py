#!/usr/bin/env python
#
# A class that mocks an AX12 servomotor.
# Holds all of the AX12 data and responds
# to all requests.
#
# This class is based on the information provided
# by Robotis at http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
# and on the information provided in the AX12 manual. Anywhere that an
# assumption has been made due to lack of information, it has been
# noted in the code.
#
# Written by Tim Peskett

import time

import const
from packet import InstPacket, StatusPacket
from utils import partition


class MemByte(object):
    """ Represents a byte in the AX12's EEPROM or 
    RAM. Contains the value, whether the value is
    R, RW, or W, and a name for debugging
    purposes. 
    """

    def __init__(self, name, default=0, ref=None, access='RW', desc=''):
            self.name = name
            self.default = default
            self.value = self.default
            self.access = access
            self.desc = desc

            self.ref = ref
            self.is_resolved = (self.ref is None)

    def reset(self):
        """Resets a byte back to its initial state. This method
        will reset the value of the byte to its default value, and
        will unresolve any references.
        """

        self.value = self.default
        self.is_resolved = (self.ref is not None)



class AX12(object):
    """ Simulates an AX12 servomotor. Takes instruction packets and returns
    status packets based on the values in its memory.
    """

    #Address at which EEPROM starts and RAM stops
    EEPROM_START = 0x12

    #The memory (EEPROM and RAM) of the AX12.
    mem = [
            MemByte('Model Number(L)', default=12, access='R', desc='Lowest byte of model number'),
            MemByte('Model Number(H)', access='R', desc='Highest byte of model number'),
            MemByte('Version of Firmware', access='R', desc='Information on the version of firmware'),
            MemByte('ID', default=1, desc='ID of Dynamixel'),
            MemByte('Baud Rate', default=1, desc='Baud Rate of Dynamixel'),
            MemByte('Return Delay Time', default=250, desc='Return Delay Time'),
            MemByte('CW Angle Limit(L)', desc='Lowest byte of clockwise Angle Limit'),
            MemByte('CW Angle Limit(H)', desc='Highest byte of clockwise Angle Limit'),
            MemByte('CCW Angle Limit(L)', default=255, desc='Lowest byte of counterclockwise Angle Limit'),
            MemByte('CCW Angle Limit(H)', default=3, desc='Highest byte of counterclockwise Angle Limit'),
            MemByte('Highest Limit Temperature', default= 70, desc='Internal Limit Temperature'),
            MemByte('Lowest Limit Voltage', default=60),
            MemByte('Highest Limit Voltage', default=140),
            MemByte('Max Torque(L)', default=255, desc='Lowest byte of Max. Torque'),
            MemByte('Max Torque(H)', default=3, desc='Highest byte of Max. Torque'),
            MemByte('Status Return Level', default=2, desc='Status Return Level'),
            MemByte('Alarm LED', default=36, desc='LED for Alarm'),
            MemByte('Alarm Shutdown', default=36, desc='Shutdown for Alarm'),
            #Padding Bytes
            MemByte('Padding#1'),
            MemByte('Padding#2'),
            MemByte('Padding#3'),
            MemByte('Padding#4'),
            MemByte('Padding#5'),
            #End padding
            MemByte('Torque Enable', desc='Torque On/Off'),
            MemByte('LED', desc='LED On/Off'),
            MemByte('CW Compliance Margin', default=1, desc='CW Compliance margin'),
            MemByte('CCW Compliance Margin', default=1, desc='CCW Compliance margin'),
            MemByte('CW Compliance Slope', default=32, desc='CW Compliance slope'),
            MemByte('CCW Compliance Slope', default=32, desc='CCW Compliance slope'),
            MemByte('Goal Position(L)', desc='Lowest byte of Goal Position'),
            MemByte('Goal Position(H)', desc='Highest byte of Goal Position'),
            MemByte('Moving Speed(L)', desc='Lowest byte of Moving Speed (Moving Velocity)'),
            MemByte('Moving Speed(H)', desc='Highest byte of Moving Speed (Moving Velocity)'),
            MemByte('Torque Limit(L)', ref=14, desc='Lowest byte of Torque Limit (Goal Torque)'),
            MemByte('Torque Limit(H)', ref=15, desc='Highest byte of Torque Limit (Goal Torque)'),
            MemByte('Present Position(L)', access='R', desc='Lowest byte of Current Position (Present Velocity)'),
            MemByte('Present Position(H)', access='R', desc='Highest byte of Current Position (Present Velocity)'),
            MemByte('Present Speed(L)', access='R', desc='Lowest byte of Current Speed'),
            MemByte('Present Speed(H)', access='R', desc='Highest byte of Current Speed'),
            MemByte('Present Load(L)', access='R', desc='Lowest byte of Current Load'),
            MemByte('Present Load(H)', access='R', desc='Highest byte of Current Load'),
            MemByte('Present Voltage', access='R', desc='Current Voltage'),
            MemByte('Present Temperature', access='R', desc='Current Temperature'),
            MemByte('Registered', access='R', desc='Means if Instruction is registered'),
            MemByte('Moving', access='R', desc='Means if there is any movement'),
            MemByte('Lock', desc='Locking EEPROM'),
            MemByte('Punch(L)', default=32, desc='Lowest byte of Punch'),
            MemByte('Punch(H)', desc='Highest byte of Punch'),
        ]


    def __init__(self, dyn_id, name=None):
        self._resolve_refs()

        self.dyn_id = dyn_id
        #Buffer used to store data when issued a regwrite command
        self.regwrite_buffer = None


    def receive(self, packet):
        """This function is long and complicated. This should be 
        taken as proof that it works, as the AX12 is also complicated.

        This function simulates the receiving of a packet
        by the AX12 object that it is called on.
        To ensure accurate simulation, this method may take some time
        to return to the caller. As such it is likely that the caller
        will want to call this function inside of a separate thread to
        reflect the fact that each AX12 has its own processor.
        Params:
            packet - A list of byte values containing the raw data of
                     an ax12 instruction packet. See the ax12 manual
                     for a description of this format.
        Returns:
            Either a status packet (in byte form) that should be sent
            back to the 'controller' or None if no status packet is 
            required.
        """
        status = None
        status_return_level = self.get_status_return()
        inst_pack = InstPacket.deserialize(packet)

        if inst_pack.checksum != inst_pack.calc_checksum():
            raise AX12ChecksumException('Bad checksum on received packet')
        if inst_pack.dyn_id != self.dyn_id and inst_pack.dyn_id != const.ID_BROADCAST:
            raise AX12Error('Instruction packet delivered to\
                             motor ' + self.dyn_id + ' instead of\
                             motor ' + inst_pack.dyn_id)


        #Determine instruction and process it 
        if inst_pack.instruction == const.INST_PING:
            #Pings always and only return a status
            status = StatusPacket(self.dyn_id, 0, [])

        elif inst_pack.instruction == const.INST_READDATA:
            params = inst_pack.params
            start_addr = params[0]
            data_len = params[1]
            read_bytes = []

            for offset in range(data_len):
                read_bytes.append(self.get_byte(start_addr + offset))

            #Return the read data if applicable
            if status_return_level != const.SRL_NONE:
                status = StatusPacket(self.dyn_id, 0, read_bytes)

        elif inst_pack.instruction == const.INST_WRITEDATA:
            #Write bytes to memory and return a status if necessary
            params = inst_pack.params
            start_addr = params[0]
            for offset, byte in enumerate(params[1:]):
                self.set_byte(start_addr + offset, byte)

            if status_return_level == const.SRL_ALL:
                status = StatusPacket(self.dyn_id, 0, [])

        elif inst_pack.instruction == const.INST_REGWRITE:
            #ASSUME: Overwrite REGWRITE buffer if new REGWRITE
            #instruction occurs before an ACTION instruction.
            self.set_byte(const.ADDR_REGISTER, 1)
            self.regwrite_buffer = inst_pack.params

            if status_return_level == const.SRL_ALL:
                status = StatusPacket(self.dyn_id, 0, [])

        elif inst_pack.instruction == const.INST_ACTION:
            params = self.regwrite_buffer
            start_addr = params[0]
            for offset, byte in enumerate(params[1:]):
                self.set_byte(start_addr + offset, byte)

            self.set_byte(const.ADDR_REGISTER, 0)

            if status_return_level == const.SRL_ALL:
                status = StatusPacket(self.dyn_id, 0, [])

        elif inst_pack.instruction == const.INST_RESET:
            #Set all bytes to default and resolve references again
            for byte in self.mem:
                byte.reset()
            self._resolve_refs()

            if status_return_level == const.SRL_ALL:
                status = StatusPacket(self.dyn_id, 0, [])

        elif inst_pack.instruction == const.INST_SYNCWRITE:
            params = inst_pack.params
            start_addr = params[0]
            single_length = params[1]
            
            #Find parameters for this dynamixel
            for my_params in partition(params[2:], single_length):
                if my_params[0] == self.dyn_id:
                    #The params are for me. Write the data!
                    for offset, byte in enumerate(my_params[1:]):
                        self.set_byte(start_addr + offset, byte)

            if status_return_level == const.SRL_ALL:
                status = StatusPacket(self.dyn_id, 0, [])

        else:
            raise AX12Exception('Bad instruction received')
    
        #Return a status if it is required
        if status is not None:
            #Sleep in microseconds
            time.sleep(self.get_delay_time() / 1000000)
            return status.serialize()
        else:
            return None


    def get_byte(self, addr):
        """Returns the value of the byte at addr. Throws an AX12Exception
        if the addr is out of range.
        """

        if addr not in range(0,len(self.mem)):
            raise AX12Exception('Address ' + hex(addr) + ' is out of range.')

        return self.mem[addr].value


    def set_byte(self, addr, val):
        """This function sets one of the bytes in memory. For the byte
        to be set, the byte must be writable.
        """
        if addr not in range(0,len(self.mem)):
            raise AX12Exception('Address ' + hex(addr) + ' is out of range.')
        if not self.mem[addr].is_writable():
            raise AX12Exception('Address ' + hex(addr) + ' is not writable.')

        self.mem[addr].value = val


    def set_byte_force(self, addr, val):
        """Sets one of the bytes in memory. This method will succeed
        regardless of whether the byte is writable or not.
        """
        if addr not in range(0,len(self.mem)):
            raise AX12Exception('Address ' + hex(addr) + ' is out of range.')

        self.mem[addr].value = val


    def get_baud(self):
        """Returns the baud rate as a baud rate
        instead of the ax12's internal value.
        """
        return 2000000.0 / (self.get_byte(const.ADDR_BAUDRATE) + 1)

    
    def get_delay_time(self):
        """Returns the delay time in microseconds. This is
        the time between receiving an instruction packet and
        sending a status packet
        """
        return self.get_byte(const.ADDR_BAUDRATE) * 2

    
    def get_status_return(self):
        return self.get_byte(const.ADDR_STATUSRETURNLEVEL)

    
    def _resolve_refs(self):
        """Resolves all references to initialise memory. Some of the bytes
        in memory must be initialised to the same value of another byte,
        this is called a 'reference'.
        Does not resolve circular dependencies. A circular dependency will
        result in undefined value for any involved bytes.
        """
        first = True
        num_resolved = 0

        try:
            #first used here to emulate a do-while loop
            while first or num_resolved != 0:
                num_resolved = 0
                first = False

                #Resolve all references that don't link to an unresolved address
                for i, byte in enumerate(self.mem):
                    if not byte.is_resolved and self.mem[byte.ref].is_resolved:
                        #Resolve reference
                        byte.value = self.mem[byte.ref].value
                        byte.is_resolved = True
                        num_resolved = num_resolved + 1

        except IndexError, i:
            raise AX12Error('Bad reference provided in memory')
        



#Exceptions that an AX12 can throw

class AX12Exception(Exception):
    """This exception exists to catch errors in the actual AX12's
    functioning. This could be caused by a bad packet, a bad value
    in memory or some other reason. Usually these exceptions can
    be ignored without much harm. At the worst, your virtual servomotor
    could damage itself (*gasp*)
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class AX12ChecksumException(AX12Exception):
    """This exception is thrown specifically when a bad checksum
    is encountered.
    """
    def __init__(self, value):
        self.value = value

class AX12Error(Exception):
    """This exception exists for errors, i.e things that should
    never really occur. If this exception is thrown then there is
    a bug.
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)



#Test Case
if __name__ == '__main__':
    ax12 = AX12(1, 'MyMotor')

    byte_string = raw_input('Enter raw packet string (comma separated hex values e.g 0xFF,0xFF,...): ')
    bytes = map(lambda byte: int(byte, 16), byte_string.split(','))
    print 'Using bytes: ', bytes

    print 'Response: ', ax12.receive(bytes)
