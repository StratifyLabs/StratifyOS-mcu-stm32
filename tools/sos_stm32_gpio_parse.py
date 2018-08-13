#!/usr/bin/python

# Copyright 2011-2018 Tyler Gilbert;
# This file is part of Stratify OS.
#
# Stratify OS is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Stratify OS is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>.

import xml.etree.ElementTree as ET
import argparse
import re
import sys
from shutil import copyfile
import pdb

MAX_PINS = 16
MAX_SIGNALS = 16

parser = argparse.ArgumentParser()
parser.add_argument("xml", help="input xml file.")

parser.add_argument("-c", type=str,
                    help="name of the output C file.")                    
args = parser.parse_args()

outfile = "core_set_alternate_func.c"
xml_file = ""

xml_file = args.xml

if args.c:
    outfile = args.c

def split_string(af_name):

    af_name = af_name.split("_")[0]
    integers = re.split("\d+", af_name)
    if len(integers) > 2:
        af_name = af_name.rstrip(integers[-1])
    text = af_name.rstrip('0123456789')
    integer = af_name[len(text):]
    return [text, integer]

translations = {
    "TIM" : "TMR",
    "COMP" : "TMR",
    "OTG" : "USB",
    "TRACE" : "JTAG",
    "USART" : "USRT",
    "LPTIM" : "LPTM",
    "SPDIF" : "SPDF",
    "I2S" : "SPI",
    "SAI" : "I2S",
    "DFSDM": "DFSD",
    "LPUART":"LUART",
    "QUADSPI": "QSPI"
}

tree = ET.parse(xml_file)
root = tree.getroot()
namespace = {'ns0': 'http://www.w3.org/2001/XMLSchema-instance',
      'default': 'http://mcd.rou.st.com/modules.php?name=mcu'}

pin_struct = {}

for pin in root.findall('default:GPIO_Pin', namespace):
    port_name = pin.attrib['PortName']
    pin_name = ''
    for tag in pin.findall('default:SpecificParameter', namespace):
        if 'GPIO_PIN_' in tag[0].text:
            try:
                pin_number = int(tag[0].text.replace('GPIO_PIN_', ''))
                pin_name = port_name + str(pin_number)
            except:
                continue
        else:
            continue

    if pin_name == "":
        continue

    try:
        pin_struct[pin_name]
    except :
        pin_struct[pin_name] = {}

    for signal in pin.findall('default:PinSignal', namespace):        
        temp_str = signal[0][0].text.split('_', 2)
        signal_number = temp_str[1].replace('AF', '')
        alt_func = split_string(temp_str[2])
        alt_func_name = split_string(signal.attrib['Name'])
        
        if alt_func[1] == '':
            alt_func[1] = '1'
        
        if alt_func_name[1] == '':
            alt_func_name[1] = '1'

        if alt_func[1] != alt_func_name[1]:
            print("-------")
            print("mismatch found for pin : " + pin_name + ", signal : " + signal_number)
            print("\tsignal name : " + signal.attrib['Name'])
            print("\tpossible value : " + signal[0][0].text)
            alt_func[1] = alt_func_name[1]            

        if  alt_func[0] in translations.keys():
            func_name = translations[alt_func[0]]
        else:
            func_name = alt_func[0]
        
        macro ="E_" + func_name  + '(' + alt_func[1]+')' 
        len_macro = len(macro)
        if len_macro == 10:
            macro ="E" + func_name  + '(' + alt_func[1]+')' 
        elif len_macro  == 8:
            macro ="E_" + func_name  + '_(' + alt_func[1]+')' 
        elif len_macro > 10:
            print("------")
            print("unknown function name : " + func_name)
            
        pin_struct[pin_name][signal_number] = macro

ports = []
for port in root.findall('default:GPIO_Port', namespace):
    ports.append(port.attrib['Name'])

'''
THE script assumes that the naming convention for port is as follows
port name = P<X> where X is a single chaaracter,
if the port name consists of more than one character the script will break.
'''
ports = sorted(ports) 
if len(ports) < 2:
    sys.exit(0)

port_start = ports[0][1:]
port_end  = ports[-1][1:]
    
pins_in_last_port = 0

for i in range(0, MAX_PINS):
    pin_name = 'P' + port_end + str(i)
    try:
        pin_struct[pin_name]
        pins_in_last_port = i + 1
    except:
        break

copyfile("pre_file.txt", outfile)

# ALL information extracted from the XML file now writing to C file.
with open(outfile, "a") as fd:
    fd.write("\n//#define MCU_TOTAL_PINS (" + str(ord(port_end) - ord(port_start)) + "*16+" + str(pins_in_last_port) + ")\n\n")
    fd.write("const alternate_function_entry_t alternate_function_table[MCU_TOTAL_PINS] = {\n")

    for i in range(ord(port_start),ord(port_end)):
        fd.write("		    //   0           1          2          3          4         5           6         7          8           9          10        11         12          13        14          15\n")
        for j in range(0, MAX_PINS):
            pin_name = 'P' + chr(i) + str(j)
            fd.write("        {{")
            for k in range(0,MAX_SIGNALS):
                if k == 15:
                    fd.write(" E_SYS_(1)")                 
                else:
                    try:
                        pin_struct[pin_name][str(k)]
                        fd.write(" " + pin_struct[pin_name][str(k)] + ",")
                    except:
                        fd.write(" RESERVED_,")                
            fd.write("}}, //" + pin_name + "\n")
        
    fd.write("        //   0           1          2          3          4         5           6         7          8           9          10        11         12          13        14          15\n")
    
    for j in range(0, pins_in_last_port):
        pin_name = 'P' + port_end + str(j)
        fd.write("        {{")
        for k in range(0,MAX_SIGNALS):
            if k == 15:
                fd.write(" E_SYS_(1)")                 
            else:
                try:
                    pin_struct[pin_name][str(k)]
                    fd.write(" " + pin_struct[pin_name][str(k)] + ",")
                except:
                    fd.write(" RESERVED_,")                
        fd.write("}}, //" + pin_name + "\n")
    fd.write("\n};\n")
