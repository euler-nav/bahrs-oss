## @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
import tkinter as tk
import json
import os
import sys
import glob
import serial
from serial.tools import list_ports
import numpy
import argparse
import threading
import time
from tkinter import LabelFrame
from tkinter import ttk
from tkinter import filedialog

#
# C++ Realization with same table:
#
# uint32_t Crc32(const unsigned char * buf, size_t len)
# {
#     uint32_t crc = 0xFFFFFFFF;
#     while (len--)
#         crc = (crc >> 8) ^ CRC32_TABLE[(crc ^ *buf++) & 0xFF];
#     return crc ^ 0xFFFFFFFF;
# }
#
def CRC32(bytes, crc32=0xFFFFFFFF):
  CRC32_TABLE = [
      0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
      0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
      0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
      0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
      0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
      0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
      0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
      0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
      0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
      0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
      0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
      0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
      0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
      0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
      0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
      0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
      0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
      0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
      0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
      0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
      0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
      0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
      0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
      0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
      0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
      0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
      0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
      0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
      0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55,
      0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
      0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28,
      0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
      0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
      0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
      0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
      0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
      0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
      0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
      0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
      0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
      0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693,
      0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
      0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
  ]

  for b in bytes:
    crc32 = (crc32 >> 8) ^ CRC32_TABLE[(crc32 ^ b) & 0xFF]

  return crc32 ^ 0xFFFFFFFF


#
# Class cDevice for interaction with device via COM-port
#


class cDevice:

  PROTOCOL_VERSION = 3
  COM_PORT_SPEED = 115200
  NVM_BLOCK_SIZE = 32

  port = None

  PACKET_TYPE_OPEN_NVM = 0xF0
  PACKET_TYPE_CLOSE_NVM = 0xF1
  PACKET_TYPE_READ_PAGE = 0xF2
  PACKET_TYPE_NVM_PAGE_DATA = 0xF3
  PACKET_TYPE_CONFIRM = 0xFF
  SERIAL_READ_TIMEOUT = 2.0

  STATE_FLASH_ERROR = -3
  STATE_WORKING = -2
  STATE_UNKNOWN = -1
  STATE_NORMAL = 0
  STATE_OPENED = 1

  def __init__(self):
    self.state = self.STATE_UNKNOWN
    self.taskProgress = 0
    self.buffer = bytes()

  #
  # Method: getSystemSerialPorts()
  # Returns a list of the serial ports available on the system
  # Raises EnvironmentError on unsupported or unknown platforms
  #

  def getSystemSerialPorts(self):
    comports = list_ports.comports()
    return comports

  #
  # Method: setCOMPort()
  # Sets current COM port to use.
  #
  def setCOMPort(self, port):
    self.port = port

  #
  # Method: openNVM
  # Open the devide in Debug Mode.
  #
  def openNVM(self):
    response = self.__sendPacket(self.PACKET_TYPE_OPEN_NVM, answer_sz=2)

    if response == None:
      self.state = self.STATE_UNKNOWN
      return

    if response[self.__getPacketHeaderSize()] == self.PACKET_TYPE_OPEN_NVM:
      self.state = self.STATE_OPENED

  #
  # Method: closeNVM
  # Close the Debug Mode (returning the device to normal operation).
  #
  def closeNVM(self):
    response = self.__sendPacket(self.PACKET_TYPE_CLOSE_NVM, answer_sz=2)

    if response == None:
      self.state = self.STATE_UNKNOWN
      return

    if response[self.__getPacketHeaderSize()] == self.PACKET_TYPE_CLOSE_NVM:
      self.state = self.STATE_NORMAL
    pass

  #
  # Method: getNVMPage
  # Download one page of NVM Data.
  # ! Device must be opened in Debug Mode first.
  #
  def getNVMPage(self, page_no):
    response = self.__sendPacket(
        self.PACKET_TYPE_READ_PAGE,
        page_no.to_bytes(1, 'little'),
        answer_sz=self.NVM_BLOCK_SIZE + 1,  # +1 page no
        answer_ptype=self.PACKET_TYPE_NVM_PAGE_DATA)
    return response
  
  #
  # Method: getNVMPagePart
  # Return the part of NVM image as packet
  # ! Device must be opened in Debug Mode first.
  #
  def getNVMPagePart(self, page_no, offset, size):
    if offset > self.NVM_BLOCK_SIZE or size > self.NVM_BLOCK_SIZE:
      return None
    
    data = self.getNVMPage(page_no)
    if data == None:
      return None
  
    
    i_header = self.__getPacketHeaderSize() + 1   # +1 is page no

    response = data[:i_header]    
    response += data[i_header + offset : i_header + offset + size]
    response += CRC32(response).to_bytes(4, 'little')               # CRC32 in downloaded packet is valid here.
                                                                    # replace it for packet part to future checks if needed
    return response


  #
  # Method: sendNVMPage
  # Upload one page of NVM Data to device.
  # ! Device must be opened in Debug Mode first.
  #
  def sendNVMPage(self, page_no, data):
    data = page_no.to_bytes(1, 'little') + data

    response = self.__sendPacket(
        self.PACKET_TYPE_NVM_PAGE_DATA,
        data,
        answer_sz=2,  # +1 page no +1 status (0/1)
        answer_ptype=self.PACKET_TYPE_CONFIRM)

    return response
  

  #
  # Method: sendNVMPagePart
  # Upload part of NVM page to device.
  # ! Device must be opened in Debug Mode first.
  #
  def sendNVMPagePart(self, page_no, data, offset):
    if offset > self.NVM_BLOCK_SIZE or len(data) > self.NVM_BLOCK_SIZE:
      return None
  
    data_old = self.getNVMPage(page_no)
    if data_old == None:
      return None
    
    data_old = data_old[self.__getPacketHeaderSize() + 1:-4]

    result = data_old[ : offset] + data + data_old[offset + len(data) : ]
    return self.sendNVMPage(page_no, result)


  #
  # Method: downloadNVM
  # Download NVM image data from device.
  # ! Device must be opened in Debug Mode first.
  #
  def downloadNVM(self, event=None, size_bytes=8000, offset=0):
    self.taskProgress = 0
    self.state = self.STATE_WORKING
    self.buffer = bytes()
    
    rd_end = size_bytes + offset

    while offset < rd_end:
      if event != None and event.is_set():
        break

      block_offset = offset % self.NVM_BLOCK_SIZE
      block_no = offset // self.NVM_BLOCK_SIZE
      response = None

      if block_offset == 0:
        size_remain = rd_end - offset
        if size_remain >= self.NVM_BLOCK_SIZE:
          response = self.getNVMPage(block_no)
          offset += self.NVM_BLOCK_SIZE
        else:
          response = self.getNVMPagePart(block_no, block_offset, size_remain)
          offset += size_remain
      else:
        size_rd = self.NVM_BLOCK_SIZE - block_offset
        if (size_rd > size_bytes):
          size_rd = size_bytes
        response = self.getNVMPagePart(block_no, block_offset, size_rd)
        offset += size_rd

      if (response == None) or (response[self.__getPacketHeaderSize()] != block_no):
        if (response != None):
          ptype = response[self.__getPacketHeaderSize()]
          pstate = response[self.__getPacketHeaderSize() + 1]

          if (ptype == self.PACKET_TYPE_READ_PAGE) and (pstate == 0xFF):
            self.state = self.STATE_FLASH_ERROR
            return

        self.state = self.STATE_UNKNOWN
        return

      self.buffer += response[self.__getPacketHeaderSize() + 1:-4]  # +1 is page number, -4 is CRC32 size
      self.taskProgress = offset / rd_end * 100

    self.taskProgress = 0
    self.state = self.STATE_OPENED

  #
  # Method: uploadNVM
  # Upload NVM image data to device, page-by-page
  # ! Device must be opened in Debug Mode first.
  #

  def uploadNVM(self, data, event=None, size_bytes=8000, offset=0):
    self.taskProgress = 0
    self.state = self.STATE_WORKING

    wr_end = size_bytes + offset
    start_offset = offset

    while offset < wr_end:
      if event != None and event.is_set():
        break

      block_offset = offset % self.NVM_BLOCK_SIZE
      block_no_dev = offset // self.NVM_BLOCK_SIZE
      data_offset = offset - start_offset
      response = None

      if block_offset == 0:
        size_remain = wr_end - offset
        if size_remain >= self.NVM_BLOCK_SIZE:
          response = self.sendNVMPage(block_no_dev, data[data_offset : data_offset + self.NVM_BLOCK_SIZE])
          offset += self.NVM_BLOCK_SIZE
        else:
          response = self.sendNVMPagePart(block_no_dev, data[data_offset : data_offset + size_remain], block_offset)
          offset += size_remain
      else:
        size_wr = self.NVM_BLOCK_SIZE - block_offset
        if (size_wr > size_bytes):
          size_wr = size_bytes
        response = self.sendNVMPagePart(block_no_dev, data[data_offset : data_offset + size_wr], block_offset)
        offset += size_wr

      if (response == None) or (response[self.__getPacketHeaderSize() + 1] != 0):
        self.state = self.STATE_UNKNOWN
        return
      
      self.taskProgress = offset / wr_end * 100

    self.taskProgress = 0
    self.state = self.STATE_OPENED


  #
  # Method: getBuffer
  # Return current buffer, containing the downloaded data from device
  # ! Device must be opened in Debug Mode first.
  #
  def getBuffer(self):
    return self.buffer

  #
  # Method: getState
  # Get last class state.
  #
  def getState(self):
    return self.state

  #
  # Method: getTaskProgress
  # Get current task progress(download or upload)
  #
  def getTaskProgress(self):
    return self.taskProgress

  #
  # Method: __getPacketHeader // Private
  # Return bytearray with packet header
  #
  def __getPacketHeader(self, packet_type: int):
    return bytearray([
        0x4E,  # Marker "N"
        0x45,  # Marker "E"
        self.PROTOCOL_VERSION & 0x00FF,  # # Protocol Version byte 1
        self.PROTOCOL_VERSION >> 8 & 0x00FF,  # Protocol Version byte 2
        packet_type & 0xFF,  # Packet type
    ])

  #
  # Method: __getPacketHeaderSize // Private
  # Return size of packet header.
  #
  def __getPacketHeaderSize(self):
    return 1 + 1 + 2 + 1  # N + E + protocol ver. + type

  #
  # Method: __sendPacket()
  # Packet wrapper according to protocol
  #

  def __sendPacket(self,
                   packet_type: int,
                   data=False,
                   answer_sz: int = 0,
                   answer_ptype=PACKET_TYPE_CONFIRM):

    if type(data) == bool:
      data_bytes = []
    elif type(data) == bytes:
      data_bytes = data
    else:
      data_bytes = data.tobytes()

    data_head = self.__getPacketHeader(packet_type)
    data_out = data_head + data_bytes if type(data) != bool else data_head
    data_out += CRC32(data_out).to_bytes(4, 'little')

    try:
      ser = serial.Serial(self.port, self.COM_PORT_SPEED, timeout=None)
      ser.write(data_out)
      response = self.__getResponse(ser, answer_ptype, answer_sz)
      ser.close()
    except Exception:
      response = None

    return response

  #
  # Wait and parse Serial response.
  # The interface must be opened before call the method.
  #
  def __getResponse(self, serial_handler, ptype, payload_sz=0):
    self.state = self.STATE_WORKING

    read_end_time = time.time() + self.SERIAL_READ_TIMEOUT
    rx_bytes = b''
    head_expected = self.__getPacketHeader(ptype)

    # Alternate packet type, if error
    head_accept = self.__getPacketHeader(self.PACKET_TYPE_CONFIRM)

    size_expected = len(head_expected) + payload_sz + 4  # + crc32
    size_accept = len(head_accept) + 2 + 4               # # +1 page no +1 status (0/1) + crc32
    p_offset = -1

    find_expected = -1
    find_accept = -1
    packet_size = 0
    while time.time() < read_end_time:
      if serial_handler.in_waiting > 0:
        rx_bytes += serial_handler.read(serial_handler.in_waiting)

        # Find the expected packet in received data
        find_expected = rx_bytes.find(head_expected)
        find_accept = rx_bytes.find(head_accept)

        # The packet was not found in the received stream, waiting for more data..
        if (find_expected == -1) and (find_accept == -1):
          continue

        # The packet was found, determine the size
        if (find_expected != -1):
          packet_size = size_expected
          p_offset = find_expected
        else:
          packet_size = size_accept
          p_offset = find_accept

        # The packet was found, but the amount of data is too small, waiting for more data..
        if len(rx_bytes) < p_offset + packet_size:
          continue
        
        # Packet founded in buffer, go to process ..
        break

    # Timeout ..
    if p_offset == -1:
      return None

    # Packet is smaller than expected (timeout!), so it isn't valid
    if len(rx_bytes) < p_offset + packet_size:
      return None

    # Packet founded, check the CRC32
    rx_bytes = rx_bytes[p_offset:p_offset + packet_size]
    crc32 = CRC32(rx_bytes[:-4])

    if crc32 != int.from_bytes(rx_bytes[-4:], byteorder='little'):
      return None

    return rx_bytes


#
# Class cNVMJSON for working with JSON files and NVM memory images.
#


class cNVMJSON():
  TYPE_TO_SIZE = {
      'int8_t': 1,
      'uint8_t': 1,
      'int16_t': 2,
      'uint16_t': 2,
      'int32_t': 4,
      'uint32_t': 4,
      'int64_t': 8,
      'uint64_t': 8,
      'char': 1,  # displayed as char
      'float': 4,
      'double': 8
  }


  TYPE_CONVERT = {
      'int8_t': numpy.int8,
      'uint8_t': numpy.uint8,
      'int16_t': numpy.int16,
      'uint16_t': numpy.uint16,
      'int32_t': numpy.int32,
      'uint32_t': numpy.uint32,
      'int64_t': numpy.int64,
      'uint64_t': numpy.uint64,
      'char': numpy.uint8,
      'float': numpy.float32,
      'double': numpy.float64,
  }

  BLOCK_VERSION = 'NmvFormatVersion'
  FIELD_CRC = 'Crc'

  def __init__(self):
    self.__json = None
    self.__head = {}
    self.__blocks = []


  #
  # Method: loadJSON()
  # Loads and parse JSON file.
  #
  def loadJSON(self, file_path):
    self.__json = None
    self.__head = {}
    self.__blocks = []
    self.json_path = file_path

    with open(file_path, "r", encoding="utf-8") as f:
      self.__json = json.load(f)
      f.close()
      self.__parseJSON()
  

  #
  # Method: isLoaded()
  # Returns True if JSON is loaded.
  #
  def isLoaded(self):
    return False if self.__json == None else True


  #
  # Method: calcBlockCRC32()
  # Calculates the CRC-32 checksum of a data block
  # Params: (int) block_id - id in blocks array
  #
  def calcBlockCRC32(self, block_id):
    if (block_id == -1):
      return 0

    tmp_bytearray = b''

    for f in self.__blocks[block_id]['fields']:
      if f['name'] == self.FIELD_CRC:
        f['value'][0] = CRC32(tmp_bytearray)
        return
      for val in f['value']:
        tmp_bytearray += val.tobytes()

  #
  # Method: getCRCField()
  # Returns the ID of the data block field containing the CRC32 checksum.
  # Params: (int) block_id - id in blocks array
  #
  def getCRCField(self, block_id):
    if (block_id == -1):
      return -1

    for fi, f in enumerate(self.__blocks[block_id]['fields']):
      if (f['name'] == self.FIELD_CRC):
        return fi

    return -1

  #
  # Method: loadNVMFile()
  # Loads binary data as values from file
  #
  def loadNVMFile(self, file_path):
    file_ver = {}
    with open(file_path, "rb") as f:
      fileContent = f.read()
      f.close()
      self.loadNVMBuffer(fileContent)


  #
  # Method: loadNVM()
  # Loads binary data as values from bytes buffer
  #
  def loadNVMBuffer(self, buffer):
    file_ver = {}
    offset = 0
    for block in self.__blocks:
      for field in block['fields']:
        field['value'] = numpy.frombuffer(
            buffer,
            dtype=self.TYPE_CONVERT[field['type']],
            count=field['length'],
            offset=offset).copy()

        offset += numpy.dtype(
            self.TYPE_CONVERT[field['type']]).itemsize * field['length']

        if block['name'] == self.BLOCK_VERSION:
          file_ver[field['name']] = field['value']

    # Check version:
    for k in file_ver:
      k_l = k.lower()
      if k_l in self.__head['version']:
        if self.__head['version'][k_l] != file_ver[k]:
          # reload JSON (to replace values to default)
          self.loadJSON(self.json_path)
          raise ValueError(
              'The JSON file version does not match the NVM file version!')

  def getCurrentData(self, fill=True):
    if self.__json == None:
      raise Exception('The JSON file doesn\'t opened!')

    out = bytes()
    for block in self.__blocks:
      for field in block['fields']:
        out += self.TYPE_CONVERT[field['type']](field['value']).tobytes()

    if fill:
      out += bytearray([255] * (int(self.__json['memory_size']) - len(out)))

    return out
  

  def getCurrentDataPart(self, offset, size):
    data = self.getCurrentData()
    return data[offset:offset+size]

  #
  # Method: saveNVM()
  # Save NVM binary file
  #
  def saveNVM(self, file_path, fill=True):
    if self.__json == None:
      raise Exception('The JSON file doesn\'t opened!')

    self.getCurrentData()

    fs = open(file_path, mode='wb')
    fs.write(self.getCurrentData())
    fs.close()

  #
  # Method: getHeader()
  # Returns a dictionary containing the header fields of the JSON's file
  #
  def getHeader(self) -> dict:
    return self.__head

  #
  # Method: getBlocks()
  # Returns a dictionary of all NVM data blocks information
  # with actual (if NVM file opened) or default values.
  #
  def getBlocks(self):
    return self.__blocks

  #
  # Method: getInfo()
  # Returns a dictionary of block / field / value information
  # Params: (int) block_id - id in blocks array
  #         (int) field_id - id in blocks's fields array
  #         (int) value_id - id in field values array ( every value is array )
  #
  def getInfo(self, block_id, field_id, value_id):
    info = {}
    if field_id == -1 and value_id == -1 and block_id == -1:
      return info

    if field_id == -1 and value_id == -1:
      info['block'] = self.__blocks[block_id]['name']
      info['offset'] = self.__blocks[block_id]['fields'][0]['offset']
      info['size'] = self.__blocks[block_id]['block_size']
    else:
      info = self.__blocks[block_id]['fields'][field_id].copy()
      del info['value']

    if value_id != -1:
      if info['length'] != 1:
        info['name'] += '[%s]' % value_id
      info['length'] = 1
      info['offset'] += value_id * self.TYPE_TO_SIZE[info['type']]
      if 'defaultValue' in info and isinstance(info['defaultValue'], list):
        info['defaultValue'] = info['defaultValue'][value_id]
    else:
      info['editable'] = False

    return info


  #
  # Method: getValue()
  # Returns a current field value
  # Params: (int) block_id - id in blocks array
  #         (int) field_id - id in blocks's fields array
  #         (int) value_id - id in field values array ( every value is array )
  #
  def getValue(self, block_id, field_id, value_id):
    if field_id == -1 and value_id == -1:
      return

    return self.__blocks[block_id]['fields'][field_id]['value'][value_id]


  #
  # Method: setValue()
  # Sets the value of a data field or data array element and recalculates the
  # CRC32 of the entire data block
  # Params: (int) block_id - id in blocks array
  #         (int) field_id - id in blocks's fields array
  #         (int) value_id - id in field values array ( every value is array )
  #
  def setValue(self, value, block_id, field_id, value_id):
    if field_id == -1 and value_id == -1:
      return

    vtype = self.__blocks[block_id]['fields'][field_id]['type']
    v = self.TYPE_CONVERT[vtype](value)
    self.__blocks[block_id]['fields'][field_id]['value'][value_id] = v
    self.calcBlockCRC32(block_id)


  #
  # Method: __parseJSON()  // private
  # Parses an inner JSON string.
  #
  # This method splits JSON into two parts: "header" and "blocks".
  # An offset from the beginning of the NVM binary data is calculated for each
  # field inside the block (data alignment is assumed to be equal to one byte).
  #
  # The current value field is also added (when opening JSON, it is filled with
  # default values)
  #
  def __parseJSON(self):
    for key in self.__json:

      # Blocks part ............................................................
      if key == 'blocks':
        offset = 0
        self.__blocks = self.__json[key]

        for bli, blc in enumerate(self.__blocks):
          bl_size = 0
          for fi, fc in enumerate(blc['fields']):
            self.__blocks[bli]['fields'][fi]['offset'] = offset

            # If the default value is known and has a list type, just fill
            # the current value field with it. If the number of elements in the
            # default value and the length do not match, fill in the value as
            # much as possible, fill in the rest with zeros if needed.

            if 'defaultValue' in fc:
              if isinstance(fc['defaultValue'], list):
                defaultValue = fc['defaultValue']

                if len(defaultValue) < fc['length']:
                  defaultValue += [0] * (fc['length'] - len(defaultValue))
                elif len(defaultValue) > fc['length']:
                  defaultValue = defaultValue[:fc['length']]

              else:
                defaultValue = [fc['defaultValue']] * fc['length']
            else:
              defaultValue = [0] * fc['length']

            fc['value'] = self.TYPE_CONVERT[fc['type']](defaultValue)  #

            if blc['name'] == self.BLOCK_VERSION:
              fc['editable'] = False
              key_lower = fc['name'].lower()
              if key_lower in self.__head['version']:
                fc['value'][0] = self.__head['version'][key_lower]
            else:
              fc['editable'] = True

            if fc['name'] == self.FIELD_CRC:
              fc['editable'] = False

            l = self.TYPE_TO_SIZE[fc['type']] * fc['length']
            offset += l
            bl_size += l

          self.__blocks[bli]['block_size'] = bl_size
          self.calcBlockCRC32(bli)

      # Header part ............................................................
      else:
        if isinstance(self.__json[key], list):
          self.__head[key] = ', '.join(self.__json[key])
        else:
          self.__head[key] = self.__json[key]


#
# Class cNVMManager - Main GUI / Console Application
#


class cNVMManager(tk.Tk):

  LOG_SUCCESS = 0
  LOG_INFO = 1
  LOG_ERR = 2

  INDICATOR_COLOR = {
      cDevice.STATE_FLASH_ERROR: '#000000',
      cDevice.STATE_WORKING: ['#FFC107', '#DC3545'],  # blinking
      cDevice.STATE_NORMAL: '#DC3545',
      cDevice.STATE_OPENED: '#28A745',
      cDevice.STATE_UNKNOWN: '#007BFF'
  }

  ACTION_BUTTONS_TEXT = {
      'btn_download': ['Download', 'Cancel'],
      'btn_upload': ['Upload', 'Cancel'],
      'btn_download_block': ['Download Block', 'Cancel'],
      'btn_upload_block': ['Upload Block', 'Cancel'],
      'btn_open_close_dev': ['Open', 'Close']
  }

  APP_TEXTS = {
    'title': 'STM32 NVM Manager',
    'c_started' : 'NVM Manager started [console mode]',
    'c_path_json' : 'path to JSON file',
    'c_path_nvm' : 'path to NVM binary file',
    'c_com_port' : 'COM port to open',
    'c_err_no_params' : 'The %s parameters are required to %s. Please specify them!',

    'err_timeout': 'Wrong device response or operation timeout!',
    'err_scan_com_failed': 'Scan Failed: %s',
    'err_json_not_loaded': 'The JSON file is not loaded!',
    'err_com_port_not_sel': 'You must select COM port to communicate with device!',
    'err_downloading': 'Error downloading NVM image! Wrong device response or communication timeout.',
    'err_downloading_block': 'Error downloading [ %s ] Wrong device response or communication timeout.!',
    'err_down_cancelled': 'The download was canceled by the user',
    'err_uploading_block': 'Error uploading block [ %s ]!',
    'err_uploading': 'Error uploading NVM image!',
    'err_upload_cancelled': 'The upload was canceled by the user. Warning: the data on the device may be corrupted!',
    'err_nvm_failed': 'Hardware failed! NVM reading error.',

    'ok_opened': 'Device opened successfully in Diagnostic mode!',
    'ok_closed': 'Diagnostic mode is closed successfully',
    'ok_wr_success_b': 'NVM image was written successfully [ %d bytes ]',
    'ok_wr_success': 'NVM image was written successfully',
    'ok_nvm_loaded_b': 'NVM image was loaded successfully [ %d bytes ]',
    'ok_nvm_loaded': 'NVM image loaded successfully',
    'ok_down_cmpl': 'Download NVM image from the device is complete!',
    'ok_down_block': 'Downloading of the [ %s ] NVM block is completed successfully.',
    'ok_load_block': 'NVM block [ %s ] is loaded successfully.',
    'ok_upload_block': 'Uploading of the [ %s ] NVM block is completed successfully.',
    'ok_upload_completed': 'Upload NVM to the device is complete!',

    'inf_opening_dev': 'Opening device in Diagnostic mode ...',
    'inf_uploading': 'Uploading NVM to device ...',
    'inf_downloading': 'Downloading NVM from device ...',
    'inf_downloading_block': 'Downloading NVM Block [ %s ] from device ...',
    'inf_wr_file': 'Write binary NVM image to %s ...',
    'inf_scan_com': 'Scanning for avaliable COM ports  ...',
    'inf_opening': 'Opening %s ...',
    'inf_download_cancelled': 'Download Cancelled.',
    'inf_transferring': 'Transferring ... %.2f %%',
    'inf_exit_diag': 'Exiting Diagnostic mode ...',

    'btn_open_json': 'Open JSON',
    'btn_open_nvm': 'Open NVM',
    'btn_save_nvm': 'Save NVM',
    'lbl_com_port': 'COM Port:',
    'lbl_block_transfer': 'Block transfer',
    'lbl_field_info': 'Field Info',
    'lbl_field_edit': 'Field Edit',
    'btn_set_value': 'Set'
  }

  def __init__(self):
    self.cmdMode = True if len(sys.argv) > 1 else False
    self.activeThread = None
    self.eventThread = None
    self.taskProgress = 0
    self.download_filename = None
    self.transfer_part = {}
    self.json = cNVMJSON()
    self.device = cDevice()
    self.buildGUI()
    self.buildArgParser()

  def _tr(self, phrase_id, arg=None):
    if phrase_id in self.APP_TEXTS:
      if arg:
        return self.APP_TEXTS[phrase_id] % arg
      else:
        return self.APP_TEXTS[phrase_id]
    else:
      return '## %s ##' % phrase_id

  #
  # Method: inConsoleMode()
  # Returns True if the program is running in console mode.
  #
  def inConsoleMode(self):
    return self.cmdMode

  #
  # Method: buildArgParser()
  # Console parameters parser. Disabled in GUI mode.
  #
  def buildArgParser(self):
    if not self.cmdMode:
      return

    parser = argparse.ArgumentParser()
    parser.add_argument('action',
                        help="action to perform",
                        choices=['port_list', 'new', 'download', 'upload'])
    parser.add_argument('-j', '--json', help=self._tr('c_path_json'))
    parser.add_argument('-n', '--nvm', help=self._tr('c_path_nvm'))
    parser.add_argument('-p', '--port', help=self._tr('c_com_port'))
    args = parser.parse_args()

    self.log(self._tr('c_started'))

    # Console: NEW BINARY FILE _________________________________________________
    if args.action == 'new':
      if args.json == None or args.nvm == None:
        self.log(self._tr('c_err_no_params', ('--json and --nvm', 'create a new binary file')), self.LOG_ERR)
        return

      self.loadJSON(args.json)
      self.saveNVM(args.nvm)

    # Console: PORT LIST _______________________________________________________
    elif args.action == 'port_list':
      self.getCOMPortsList()

    # Console: DOWNLOAD ________________________________________________________
    elif args.action == 'download':
      if args.json == None or args.nvm == None or args.port == None:
        self.log(self._tr('c_err_no_params', ('--json, --nvm and --port', 'download binary file from device')), self.LOG_ERR)
        return

      self.loadJSON(args.json)

      self.log(self._tr('inf_opening_dev'))
      self.device.setCOMPort(args.port)
      self.device.openNVM()

      if self.device.getState() != cDevice.STATE_OPENED:
        self.log(self._tr('err_timeout'), self.LOG_ERR)
        return
      self.log(self._tr('err_timeout'), self.LOG_SUCCESS)

      self.log(self._tr('inf_downloading'))
      self.device.downloadNVM(size_bytes=self.json.getHeader()['memory_size'])
      buffer = self.device.getBuffer()

      try:
        with open(args.nvm, "wb") as f:
          self.log(self._tr('inf_wr_file', args.nvm))
          f.write(buffer)
          self.log(self._tr('ok_wr_success_b', len(buffer)), self.LOG_SUCCESS)
          f.close()
      except Exception as e:
        self.log(e, self.LOG_ERR)

      self.log(self._tr('inf_exit_diag'))
      self.device.closeNVM()
      if self.device.getState() != cDevice.STATE_NORMAL:
        self.log(self._tr('err_timeout'), self.LOG_ERR)
        return
      self.log(self._tr('ok_closed'), self.LOG_SUCCESS)

    # Console: UPLOAD __________________________________________________________
    elif args.action == 'upload':
      if args.nvm == None or args.port == None:
        self.log(self._tr('c_err_no_params', ('--nvm and --port', 'upload binary file to device')), self.LOG_ERR)
        return

      self.log(self._tr('inf_opening'))
      self.device.setCOMPort(args.port)
      self.device.openNVM()

      if self.device.getState() != cDevice.STATE_OPENED:
        self.log(self._tr('err_timeout'), self.LOG_ERR)
        return
      self.log(self._tr('err_timeout'), self.LOG_SUCCESS)

      buffer = bytes()

      try:
        with open(args.nvm, "rb") as f:
          buffer = f.read()
          self.log(self._tr('ok_nvm_loaded_b', len(buffer)), self.LOG_SUCCESS)
          f.close()
      except Exception as e:
        self.log(e, self.LOG_ERR)

      self.log(self._tr('inf_uploading'))
      self.device.uploadNVM(buffer)

      if self.device.getState() == cDevice.STATE_UNKNOWN:
        self.log(self._tr('err_timeout'), self.LOG_ERR)
        return

      self.log(self._tr('inf_exit_diag'))
      self.device.closeNVM()
      if self.device.getState() != cDevice.STATE_NORMAL:
        self.log(self._tr('err_timeout'), self.LOG_ERR)
        return
      self.log(self._tr('ok_closed'), self.LOG_SUCCESS)

  #
  # Method: buildGUI()
  # Creates application GUI elements, binds GUI element events to handlers.
  #
  def buildGUI(self):
    if self.cmdMode:
      return

    tk.Tk.__init__(self)
    self.varsFieldVal = tk.StringVar()

    self.wm_title(self._tr('title'))
    self.columnconfigure(0, weight=1)
    self.rowconfigure(0, weight=1)

    # Main Frame
    self.fMain = ttk.Frame()
    self.fMain.grid(column=0, row=0, sticky="NSEW", padx=5, pady=5)
    self.fMain.columnconfigure(0, weight=1)
    self.fMain.columnconfigure(1, weight=0)
    self.fMain.columnconfigure(2, weight=0)
    self.fMain.rowconfigure(0, weight=0)
    self.fMain.rowconfigure(1, weight=2)
    self.fMain.rowconfigure(2, weight=0)

    # Row 0 @ Main Frame :: Buttons
    self.fButtons = ttk.Frame(self.fMain)
    self.fButtons.grid(column=0,
                       row=0,
                       columnspan=3,
                       sticky="NSEW",
                       padx=5,
                       pady=5)

    self.fsBtnFile = LabelFrame(self.fButtons, text="File", padx=5, pady=5)
    self.fsBtnDevice = LabelFrame(self.fButtons, text="Device", padx=5, pady=5)
    self.fsBtnFile.grid(column=0, row=0, sticky="NW", padx=5, pady=5)
    self.fsBtnDevice.grid(column=1, row=0, sticky="NE", padx=5, pady=5)

    self.canvStateIndicator = tk.Canvas(self.fsBtnDevice, width=22, height=22)
    self.figState = self.canvStateIndicator.create_rectangle(6, 6, 18, 18)

    self.bOpenJSON = ttk.Button(self.fsBtnFile,
                                text=self._tr('btn_open_json'),
                                command=self.h_OpenJSON)
    self.bOpenNVM = ttk.Button(self.fsBtnFile,
                               text=self._tr('btn_open_nvm'),
                               command=self.h_OpenNVM)
    self.bSaveNVM = ttk.Button(self.fsBtnFile,
                               text=self._tr('btn_save_nvm'),
                               command=self.h_SaveNVM)
    self.bDevDownload = ttk.Button(
        self.fsBtnDevice,
        text=self.ACTION_BUTTONS_TEXT['btn_download'][0],
        command=self.h_Download)
    self.bDevUpload = ttk.Button(self.fsBtnDevice,
                                 text=self.ACTION_BUTTONS_TEXT['btn_upload'][0],
                                 command=self.h_Upload)
    self.bDevOpenClose = ttk.Button(self.fsBtnDevice,
                                    text=self.ACTION_BUTTONS_TEXT['btn_open_close_dev'][0],
                                    command=self.h_OpenCloseDevice)

    self.lblPort = ttk.Label(self.fsBtnDevice, text=self._tr('lbl_com_port'))
    self.cmbCOM = ttk.Combobox(self.fsBtnDevice, state="readonly")

    self.bOpenJSON.pack(side="left", padx=5, pady=5)
    self.bOpenNVM.pack(side="left", padx=5, pady=5)
    self.bSaveNVM.pack(side="left", padx=5, pady=5)
    self.lblPort.pack(side="left", padx=5, pady=5)
    self.cmbCOM.pack(side="left", padx=5, pady=5)
    self.canvStateIndicator.pack(side="left", padx=2, pady=5)
    self.bDevOpenClose.pack(side="left", padx=5, pady=5)
    self.bDevUpload.pack(side="left", padx=5, pady=5)
    self.bDevDownload.pack(side="left", padx=5, pady=5)

    # Row 1 @ Main Frame :: JSON And Edit
    # Column 0
    self.trJSON = ttk.Treeview(self.fMain, selectmode="browse", show="tree")
    self.trJSON.grid(column=0, row=1, sticky="NSEW", padx=5, pady=5)

    self.scJSON = ttk.Scrollbar(self.fMain,
                                orient='vertical',
                                command=self.trJSON.yview)
    self.scJSON.configure(command=self.trJSON.yview)
    self.trJSON.configure(yscrollcommand=self.scJSON.set)

    # Column 1 :: JSON Scrollbar
    self.scJSON.grid(column=1, row=1, sticky='NSE')

    # Column 2 :: Info & Edit
    self.fEdit = ttk.Frame(self.fMain)
    self.fEdit.rowconfigure(0, weight=0)
    self.fEdit.rowconfigure(1, weight=1)
    self.fEdit.rowconfigure(2, weight=0)
    self.fEdit.columnconfigure(0, weight=1)

    self.fEdit.grid(column=2, row=1, sticky="NSEW", padx=5, pady=5)

    # - Row 0 fEdit
    self.fsBlockTransfer = LabelFrame(self.fEdit, text=self._tr('lbl_block_transfer'), padx=5, pady=5)
    self.fsBlockTransfer.rowconfigure(0, weight=1)
    self.fsBlockTransfer.columnconfigure(0, weight=1)
    self.fsBlockTransfer.columnconfigure(1, weight=1)
    self.fsBlockTransfer.grid(column=0, row=0, sticky="NSEW", padx=5, pady=5)

    self.bDevBlockDownload = ttk.Button(self.fsBlockTransfer, text=self.ACTION_BUTTONS_TEXT['btn_download_block'][0], command=self.h_DownloadBlock)
    self.bDevBlockUpload = ttk.Button(self.fsBlockTransfer, text=self.ACTION_BUTTONS_TEXT['btn_upload_block'][0], command=self.h_UploadBlock)
    self.bDevBlockDownload.grid(column=0, row=0, sticky="NSEW", padx=5, pady=5)
    self.bDevBlockUpload.grid(column=1, row=0, sticky="NSEW", padx=5, pady=5)
   

    # - Row 1 fEdit
    self.fsInfo = LabelFrame(self.fEdit, text=self._tr('lbl_field_info'), padx=5, pady=5)
    self.fsInfo.rowconfigure(0, weight=1)
    self.fsInfo.columnconfigure(0, weight=1)

    self.fsInfo.grid(column=0, row=1, sticky="NSEW", padx=5, pady=5)

    self.lblFieldInfo = ttk.Label(self.fsInfo)
    self.lblFieldInfo.grid(column=0, row=0, sticky="NW", padx=5, pady=5)

    # - Row 2 fEdit
    self.fsEdit = LabelFrame(self.fEdit, text=self._tr('lbl_field_edit'), padx=5, pady=5)
    self.fsEdit.rowconfigure(0, weight=1)
    self.fsEdit.columnconfigure(0, weight=4)
    self.fsEdit.columnconfigure(1, weight=0)
    self.fsEdit.grid(column=0, row=2, sticky="NSEW", padx=5, pady=5)

    self.eEdit = ttk.Entry(self.fsEdit, textvariable=self.varsFieldVal)
    self.bSetValue = ttk.Button(self.fsEdit,
                                text=self._tr('btn_set_value'),
                                command=self.h_SetValue)
    self.eEdit.grid(column=0, row=0, sticky="NSEW", padx=5, pady=5, ipady=1)
    self.bSetValue.grid(column=1, row=0, sticky="NW", padx=5, pady=5)

    # Row 2 @ Main Frame :: Logger
    self.trLog = ttk.Treeview(self.fMain,
                              selectmode="browse",
                              show="headings",
                              columns=("Type", "Time", "Message"))

    self.trLog.column("Type",
                      minwidth=25,
                      width=25,
                      stretch=False,
                      anchor='center')
    self.trLog.column("Time", minwidth=55, width=55, stretch=False)
    self.trLog.heading("Type")
    self.trLog.heading("Time", text='Time')
    self.trLog.heading("Message", text="Message")

    self.trLog.grid(column=0,
                    row=2,
                    columnspan=3,
                    sticky="NSEW",
                    padx=5,
                    pady=5)

    # Logger colors
    self.trLog.tag_configure(self.LOG_SUCCESS, foreground='#28A745')
    self.trLog.tag_configure(self.LOG_ERR, foreground='#DC3545')
    self.trLog.tag_configure(self.LOG_INFO, foreground='#007BFF')

    # Handlers bind
    self.trJSON.bind("<<TreeviewSelect>>", self.h_TreeClick)
    self.cmbCOM.bind("<<ComboboxSelected>>", self.h_COMPortSelected)

    # Leave GUI disabled
    self.__updGUIState()
    self.bSetValue.config(state=tk.DISABLED)
    self.eEdit.config(state=tk.DISABLED)

    self.log('NVM Manager started.')

    # update COM port list at startup
    self.cmbCOM['values'] = self.getCOMPortsList()

    # Start indicator update
    self.__updateIndicatorState()

  #
  # Method: __setGUIState() // Private
  # Sets the config of the GUI elements depending on the current state.
  # Params: (string) st - GUI mode: {normal, disabled}
  #

  def __updGUIState(self, state=cDevice.STATE_UNKNOWN):
    if self.cmdMode:
      return

    # Btn upload / Btn download
    if state == cDevice.STATE_OPENED:
      self.bDevDownload.config(state=tk.NORMAL)
      self.bDevUpload.config(state=tk.NORMAL)
    elif state in [cDevice.STATE_UNKNOWN, cDevice.STATE_NORMAL, cDevice.STATE_FLASH_ERROR]:
      self.bDevDownload.config(state=tk.DISABLED)
      self.bDevUpload.config(state=tk.DISABLED)
    elif (state == cDevice.STATE_WORKING) and (self.activeThread != None):
      if "download" in self.activeThread.name:
        self.bDevUpload.config(state=tk.DISABLED)
      else:
        self.bDevDownload.config(state=tk.DISABLED)

    self.__updateBlockButtonsState()

    # Btn open / close
    if state == cDevice.STATE_OPENED:
      self.bDevOpenClose.config(text=self.ACTION_BUTTONS_TEXT['btn_open_close_dev'][1])

    if state in [cDevice.STATE_NORMAL, cDevice.STATE_UNKNOWN, cDevice.STATE_FLASH_ERROR]:
      self.bDevOpenClose.config(text=self.ACTION_BUTTONS_TEXT['btn_open_close_dev'][0])

    if state == cDevice.STATE_WORKING:
      self.bDevOpenClose.config(state=tk.DISABLED)
    else:
      self.bDevOpenClose.config(state=tk.NORMAL)

    # COM Port combo
    if state in [cDevice.STATE_NORMAL, cDevice.STATE_UNKNOWN]:
      self.cmbCOM.config(state=tk.NORMAL)
    else:
      self.cmbCOM.config(state=tk.DISABLED)


  def __updateBlockButtonsState(self):
    (block_id, field_id, value_id) = self.__getTreeSelectedID()
    info = self.json.getInfo(block_id, field_id, value_id)
    
    if 'block' in info:
      st = self.device.getState()
      if st == cDevice.STATE_OPENED:
        self.bDevBlockDownload.config(state=tk.NORMAL)
        self.bDevBlockUpload.config(state=tk.NORMAL)
        return
    
    self.bDevBlockDownload.config(state=tk.DISABLED)
    self.bDevBlockUpload.config(state=tk.DISABLED)

  #
  # Method: __updateIndicatorState() // Private
  # Updates GUI indicator state every 50ms.
  #
  def __updateIndicatorState(self):
    st = self.device.getState()
    if st == cDevice.STATE_WORKING:
      curr_color = self.INDICATOR_COLOR[st][
          0] if self.canvStateIndicator.itemcget(
              self.figState, "fill"
          ) == self.INDICATOR_COLOR[st][1] else self.INDICATOR_COLOR[st][1]
    else:
      curr_color = self.INDICATOR_COLOR[st]

    self.canvStateIndicator.itemconfig(self.figState, fill=curr_color)
    self.__updGUIState(st)
    self.after(50, lambda: self.__updateIndicatorState())

  #
  # Method: getCOMPortsList()
  # Return COM Ports device list
  #
  def getCOMPortsList(self):
    self.log(self._tr('inf_scan_com'))
    try:
      comports = self.device.getSystemSerialPorts()
      self.log(
          'Discovered: \n' +
          '\n'.join(["%s - %s" % (x.device, x.description) for x in comports]),
          self.LOG_SUCCESS)
      return [x.device for x in comports]
    except OSError as e:
      self.log(self._tr('err_scan_com_failed', str(e)), self.LOG_ERR)
      return []

  #
  # Method: loadJSON()
  # Loads a JSON file and updates the tree in the GUI
  #
  def loadJSON(self, filename):
    self.log(self._tr('inf_opening', filename))
    try:
      self.json.loadJSON(filename)
      self.log('JSON file loaded successfully', self.LOG_SUCCESS)
      h = self.json.getHeader()
      json_h = []
      for k in h:
        json_h.append('%s: %s' % (k, h[k]))
      self.log('%s JSON FILE HEADER %s\n%s' %
               ('*' * 15, '*' * 15, '\n'.join(json_h)))
    except Exception as e:
      self.log(e, self.LOG_ERR)
    finally:
      self.updateTree()

  #
  # Method: loadNVM()
  # Loads a NVM binary file and updates the tree in the GUI
  #
  def loadNVM(self, filename):
    self.log(self._tr('inf_opening', filename))
    try:
      self.json.loadNVMFile(filename)
      self.log(self._tr('ok_nvm_loaded'), self.LOG_SUCCESS)
    except Exception as e:
      self.log(e, self.LOG_ERR)
    finally:
      self.updateTree()

  #
  # Method: saveNVM()
  # Saves NVM binary file
  #
  def saveNVM(self, filename):
    self.log(self._tr('inf_wr_file', filename))
    try:
      self.json.saveNVM(filename)
      self.log(self._tr('ok_wr_success'), self.LOG_SUCCESS)
    except Exception as e:
      self.log(e, self.LOG_ERR)

  #
  # Method: updateTree()
  # Updates the tree of block elements in the GUI.
  #
  def updateTree(self):
    if self.cmdMode:
      return
    
    tsel = self.__getTreeSelectedID()

    self.trJSON.delete(*self.trJSON.get_children())

    for bli, blc in enumerate(self.json.getBlocks()):
      tid_bl = self.trJSON.insert('',
                                  tk.END,
                                  iid='%i:-1:-1' % bli,
                                  text=blc['name'],
                                  open=(bli == tsel[0]))
      
      for fli, flc in enumerate(blc['fields']):
        if flc['name'] == cNVMJSON.FIELD_CRC:
          self.trJSON.insert(
              tid_bl,
              tk.END,
              iid='%i:%i:0' % (bli, fli),
              open=(fli == tsel[1]),
              text='%s: %s' %
              (flc['name'], '0x{0:0{1}X}'.format(flc['value'][0], 8)))
          pass
        else:
          if len(flc['value']) == 1:
            self.trJSON.insert(tid_bl,
                               tk.END,
                               iid='%i:%i:0' % (bli, fli),
                               text='%s: %s' % (flc['name'], flc['value'][0]))
          else:
            tid_f = self.trJSON.insert(tid_bl,
                                       tk.END,
                                       iid='%i:%i:-1' % (bli, fli),
                                       text=flc['name'])
            for vi, vc in enumerate(flc['value']):
              self.trJSON.insert(tid_f,
                                 tk.END,
                                 iid='%i:%i:%i' % (bli, fli, vi),
                                 text='[%i]: %s' % (vi, vc))

  #
  # Method: log()
  # Method of adding a message to the log. The message is displayed in a special
  # log window in GUI mode and as text in console mode.
  # Params: (string) msg - A message to display
  #         (int) level  - Message type: { self.LOG_INFO,
  #                                        self.LOG_ERR,
  #                                        self.LOG_SUCCESS }
  #
  def log(self, msg, level=LOG_INFO):
    level2tag = {
        self.LOG_ERR: '[ ERR ]' if self.cmdMode else '\u274C',
        self.LOG_INFO: '[ INF ]' if self.cmdMode else '\u2139',
        self.LOG_SUCCESS: '[  OK ]' if self.cmdMode else '\u2705'
    }

    ts = time.strftime('%H:%M:%S', time.localtime())
    tag = level2tag[level]
    msg = str(msg).split('\n')
    for m in msg:
      if self.cmdMode:
        print('%s %s' % (tag, m))
      else:
        self.trLog.insert("", tk.END, values=(tag, ts, m), tags=level)
      ts = ''
      tag = ' ' * 7

    if not self.cmdMode:
      self.trLog.yview_moveto(1)

  #
  # Method: threadMonitor()
  # Check the active thread status and call progress / callback functions.
  #

  def threadMonitor(self, finish, progress=None):
    if self.activeThread != None and self.activeThread.is_alive():
      # check the thread every 100ms
      self.after(100, lambda: self.threadMonitor(finish, progress))
      if progress != None:
        progress()
    else:
      finish()

  #
  # Method: h_TreeClick()
  # GUI Tree elements click handler
  #
  def h_TreeClick(self, event):
    (block_id, field_id, value_id) = self.__getTreeSelectedID()
    
    info = self.json.getInfo(block_id, field_id, value_id)

    self.__updateBlockButtonsState()

    info_s = ''
    for k in info:
      if k != 'editable':
        info_s += '%s: %s\n' % (k, info[k])

    self.lblFieldInfo['text'] = info_s

    if ('editable' in info) and info['editable']:
      self.varsFieldVal.set(self.json.getValue(block_id, field_id, value_id))
      self.bSetValue.config(state=tk.NORMAL)
      self.eEdit.config(state=tk.NORMAL)
    else:
      self.bSetValue.config(state=tk.DISABLED)
      self.eEdit.config(state=tk.DISABLED)
      self.varsFieldVal.set('')

  #
  # Method: h_OpenJSON()
  # Button "Open JSON" click handler
  #
  def h_OpenJSON(self):
    filetypes = (('JSON files (*.json)', '*.json'), ('All files', '*.*'))

    filename = filedialog.askopenfilename(title='Open a JSON config',
                                          initialdir=os.getcwd(),
                                          filetypes=filetypes)
    if filename:
      self.loadJSON(filename)

  #
  # Method: h_OpenNVM()
  # Button "Open NVM" click handler
  #
  def h_OpenNVM(self):
    filetypes = (('Binary files (*.bin)', '*.bin'), ('All files', '*.*'))

    filename = filedialog.askopenfilename(title='Open a binary NVM',
                                          initialdir=os.getcwd(),
                                          filetypes=filetypes)
    if filename:
      self.loadNVM(filename)

  #
  # Method: h_SaveNVM()
  # Button "Save NVM" click handler
  #
  def h_SaveNVM(self):
    filetypes = (('Binary files (*.bin)', '*.bin'), ('All files', '*.*'))

    filename = filedialog.asksaveasfilename(title='Save NVM binary file',
                                            defaultextension="*.*",
                                            initialdir=os.getcwd(),
                                            filetypes=filetypes)

    if filename:
      self.saveNVM(filename)

  #
  # Method: h_Upload()
  # Button "Upload" clock handler
  #

  def h_Upload(self):
    if not self.json.isLoaded():
      self.log(self._tr('err_json_not_loaded'), self.LOG_ERR)
      return

    self.taskProgress = 0

    if self.bDevUpload['text'] == self.ACTION_BUTTONS_TEXT['btn_upload'][0]:
      pass

    data = self.json.getCurrentData()
    if self.bDevUpload['text'] == self.ACTION_BUTTONS_TEXT['btn_upload'][0]:
      if self.device.getState() == cDevice.STATE_WORKING:
        return
      self.bDevUpload['text'] = self.ACTION_BUTTONS_TEXT['btn_upload'][1]

      self.eventThread = threading.Event()
      self.activeThread = threading.Thread(target=self.device.uploadNVM,
                                           args=(
                                               data,
                                               self.eventThread,
                                           ))
      self.log(self._tr('inf_uploading'))
      self.activeThread.start()
      self.threadMonitor(self.h_UploadResponse, self.h_DeviceProgress)
    else:
      self.activeThread = None
      self.eventThread.set()
      

  #
  # Method: h_Upload()
  # Button "Upload" clock handler
  #

  def h_Download(self):
    if not self.json.isLoaded():
      self.log(self._tr('err_json_not_loaded'), self.LOG_ERR)
      return

    size_from_json = self.json.getHeader()['memory_size']
    self.taskProgress = 0

    if self.bDevDownload['text'] == self.ACTION_BUTTONS_TEXT['btn_download'][0]:
      if self.device.getState() == cDevice.STATE_WORKING:
        return

      self.eventThread = threading.Event()
      self.activeThread = threading.Thread(target=self.device.downloadNVM,
                                           args=(
                                               self.eventThread,
                                               size_from_json,
                                           ))
      
      filetypes = (('Binary files (*.bin)', '*.bin'), ('All files', '*.*'))
      self.download_filename = filedialog.asksaveasfilename(title='Save NVM binary file',
                                                            defaultextension="*.*",
                                                            initialdir=os.getcwd(),
                                                            filetypes=filetypes)
      
      if self.download_filename:
        self.bDevDownload['text'] = self.ACTION_BUTTONS_TEXT['btn_download'][1]
        self.log(self._tr('inf_downloading'))

        self.activeThread.start()
        self.threadMonitor(self.h_DownloadResponse, self.h_DeviceProgress)
      else:
        self.log(self._tr('inf_download_cancelled'))
    else:
      self.activeThread = None
      self.eventThread.set()


  #
  #
  #
  def h_DownloadBlock(self):
    if not self.json.isLoaded():
      self.log(self._tr('err_json_not_loaded'), self.LOG_ERR)
      return
    
    (block_id, field_id, value_id) = self.__getTreeSelectedID()
    info = self.json.getInfo(block_id, field_id, value_id)
    
    if self.device.getState() == cDevice.STATE_WORKING:
      return
    
    self.transfer_part = {
      'offset': info['offset'],
      'size': info['size'] ,
      'block_name': info['block']
    }

    self.eventThread = threading.Event()
    self.activeThread = threading.Thread(target=self.device.downloadNVM,
                                          args=(
                                              self.eventThread,
                                              self.transfer_part['size'],
                                              self.transfer_part['offset'],
                                          ))
    
    self.log(self._tr('inf_downloading_block', self.transfer_part['block_name']))
    self.activeThread.start()
    self.threadMonitor(self.h_PartialDownloadResponse, self.h_DeviceProgress)


  #
  #
  #
  def h_UploadBlock(self):
    if not self.json.isLoaded():
      self.log(self._tr('err_json_not_loaded'), self.LOG_ERR)
      return
    
    (block_id, field_id, value_id) = self.__getTreeSelectedID()
    info = self.json.getInfo(block_id, field_id, value_id)

    if self.device.getState() == cDevice.STATE_WORKING:
      return
    
    self.transfer_part = {
      'offset': info['offset'],
      'size': info['size'] ,
      'block_name': info['block']
    }

    data = self.json.getCurrentDataPart(info['offset'], info['size'])

    self.eventThread = threading.Event()
    self.activeThread = threading.Thread(target=self.device.uploadNVM,
                                          args=(
                                              data,
                                              self.eventThread,
                                              self.transfer_part['size'],
                                              self.transfer_part['offset'],
                                          ))
    
    self.log('Uploading NVM Block [ %s ] ...' % self.transfer_part['block_name'])
    self.activeThread.start()
    self.threadMonitor(self.h_PartialUploadResponse, self.h_DeviceProgress)


  #
  # Method: h_OpenCloseDevice
  # Button "Open" in "Device" section handler
  #

  def h_OpenCloseDevice(self):
    port = self.cmbCOM.get()
    if port == "":
      self.log(self._tr('err_com_port_not_sel'), self.LOG_ERR)
      return

    self.device.setCOMPort(port)
    try:
      if self.device.getState() == cDevice.STATE_OPENED:
        self.log(self._tr('inf_exit_diag'))
        self.activeThread = threading.Thread(target=self.device.closeNVM)
      else:
        self.log(self._tr('inf_opening_dev'))
        self.activeThread = threading.Thread(target=self.device.openNVM)
      self.activeThread.start()
      self.threadMonitor(self.h_OpenCloseDeviceResponse)
    except Exception as e:
      self.log(e, self.LOG_ERR)


  #
  # Method: h_OpenCloseDeviceResponse()
  # Device response handler.
  # Called when the com port response waiting thread is completed.
  #
  def h_OpenCloseDeviceResponse(self):
    if self.device.getState() == cDevice.STATE_OPENED:
      self.log(self._tr('ok_opened'), self.LOG_SUCCESS)
    elif self.device.getState() == cDevice.STATE_NORMAL:
      self.log(self._tr('ok_closed'), self.LOG_SUCCESS)
    else:
      self.log(self._tr('err_timeout'), self.LOG_ERR)


  #
  # Method: h_OpenCloseDeviceResponse()
  # Device response handler.
  # Called when the com port response waiting thread is completed.
  #
  def h_DownloadResponse(self):
    self.bDevDownload['text'] = self.ACTION_BUTTONS_TEXT['btn_download'][0]
    if self.device.getState() == cDevice.STATE_UNKNOWN:
      self.log(self._tr('err_downloading'), self.LOG_ERR)
      return

    if self.device.getState() == cDevice.STATE_FLASH_ERROR:
      self.log(self._tr('err_nvm_failed'), self.LOG_ERR)
      return

    buffer = self.device.getBuffer()

    if self.activeThread == None:
      self.log(self._tr('err_down_cancelled'), self.LOG_ERR)
      return

    self.log(self._tr('ok_down_cmpl'), self.LOG_SUCCESS)

    if self.download_filename:
      with open(self.download_filename, "wb") as f:
        self.log(self._tr('inf_wr_file', self.download_filename))
        try:
          f.write(buffer)
          self.log(self._tr('ok_wr_success_b', len(buffer)), self.LOG_SUCCESS)
        except Exception as e:
          self.log(e, self.LOG_ERR)
        finally:
          f.close()

    try:
      self.json.loadNVMBuffer(buffer)
      self.log(self._tr('ok_nvm_loaded'), self.LOG_SUCCESS)
    except Exception as e:
      self.log(e, self.LOG_ERR)


  def h_PartialDownloadResponse(self):
    if self.device.getState() == cDevice.STATE_UNKNOWN:
      self.log(self._tr('err_downloading_block', self.transfer_part['block_name']), self.LOG_ERR)
      return
    
    self.log(self._tr('ok_down_block', self.transfer_part['block_name']), self.LOG_SUCCESS)

    buffer = self.device.getBuffer()
    data = self.json.getCurrentData()
    buffer_result = data[:self.transfer_part['offset']] + buffer + data[self.transfer_part['offset'] + self.transfer_part['size']:]

    try:
      self.json.loadNVMBuffer(buffer_result)
      self.log(self._tr('ok_load_block', self.transfer_part['block_name']), self.LOG_SUCCESS)
    except Exception as e:
      self.log(e, self.LOG_ERR)

    self.updateTree()


  def h_PartialUploadResponse(self):
    if self.device.getState() == cDevice.STATE_UNKNOWN:
      self.log(self._tr('err_uploading_block', self.transfer_part['block_name']), self.LOG_ERR)
      return
    
    self.log(self._tr('ok_upload_block', self.transfer_part['block_name']), self.LOG_SUCCESS)

  #
  # Method: h_UploadResponse()
  # Device upload complete handler.
  # Called when the com port response waiting thread is completed.
  #
  def h_UploadResponse(self):
    self.bDevUpload['text'] = self.ACTION_BUTTONS_TEXT['btn_upload'][0]

    if self.device.getState() == cDevice.STATE_UNKNOWN:
      self.log(self._tr('err_uploading'), self.LOG_ERR)
      return

    if self.activeThread == None:
      self.log(self._tr('err_upload_cancelled'), self.LOG_ERR)
      return

    self.log(self._tr('ok_upload_completed'), self.LOG_SUCCESS)


  #
  # Method: h_DeviceProgress()
  # Device action (upload or download) progress handler.
  #
  def h_DeviceProgress(self):
    progress = self.device.getTaskProgress()
    if (progress != self.taskProgress) and (progress > 0):
      self.log(self._tr('inf_transferring', progress))
      self.taskProgress = progress

  #
  # Method: h_SetValue()
  # Button "Set" click handler
  #
  def h_SetValue(self):
    try:
      (block_id, field_id, value_id) = self.__getTreeSelectedID()
      self.json.setValue(self.eEdit.get(), block_id, field_id, value_id)

      self.updateTreeValue(block_id, field_id, value_id)
      crcField = self.json.getCRCField(block_id)
      if (crcField != -1):
        self.updateTreeValue(block_id, crcField, 0, hex=True)

    except Exception as e:
      self.log(e, self.LOG_ERR)

  #
  # Method: h_COMPortSelected()
  # COM Port selection handler
  #
  def h_COMPortSelected(self, event):
    selection = self.cmbCOM.get()
    st = 'normal' if selection else 'disabled'

  #
  # Method: updateTreeValue()
  # The method loads data from the JSON class and updates the tree element.
  # Params: (int) block_id - id in blocks array
  #         (int) field_id - id in blocks's fields array
  #         (int) value_id - id in field values array ( every value is array )
  #         (bool) hex     - Display value as 4byte hexadecimal (for CRC32)
  #
  def updateTreeValue(self, block_id, field_id, value_id, hex=False):
    val = self.json.getValue(block_id, field_id, value_id)
    tree_id = '%i:%i:%i' % (block_id, field_id, value_id)
    name = self.trJSON.item(tree_id)['text'].split(':')[0]
    if hex:
      val = '0x{0:0{1}X}'.format(val, 8)

    self.trJSON.item(tree_id, text='%s: %s' % (name, val))

  #
  # Method: __getTreeSelectedID() // Private
  # Parser ID of the selected element in the GUI tree.
  # Returns three values: (block id, field id, and data array element id inside
  # the field).
  #
  def __getTreeSelectedID(self):
    return [int(x) for x in self.__getTreeSelected().split(':', 3)]

  #
  # Method: __getTreeSelected() // Private
  # Returns ID of the selected element in the GUI tree.
  #
  def __getTreeSelected(self):
    sel = self.trJSON.selection()
    return self.trJSON.selection()[0] if sel else '-1:-1:-1'


################################################################################
################################################################################

if __name__ == "__main__":
  App = cNVMManager()
  if not App.inConsoleMode():
    App.mainloop()
