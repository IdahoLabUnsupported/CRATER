'''
    Copyright 2025, Battelle Energy Alliance, LLC, ALL RIGHTS RESERVED
    
    Limited interface for BDM devices, to be used with an Arduino Uno programmed with the ard_bdm2.ino sketch
    Does reading/writing to/from addresses and registers, can dump a range of memory and fill a chunk of memory with a given file
    
    Does not do any source code tracing like a true debugger. 
    It's possible to implement a crude version of that by manually watching the Program counter, 
    but it is impossible (at least on the device used to develop this) to do this accurately. 
        The program counter jumps forward an arbitrary number of instructions per "step".

    The main use for this program was to dump the firmware from the attached BDM device, and write to it

    Pinout diagram can be found https://cmp.felk.cvut.cz/~pisa/m683xx/bdm_driver.html as well as by running the "pinout" command.

'''
import serial, sys
import subprocess
from enum import Enum
from rich.console import Console
from rich.table import Table
from rich.align import Align
from rich.columns import Columns
from rich.panel import Panel
from rich.progress import track
from rich.padding import Padding
from rich import box
from prompt_toolkit import PromptSession
from prompt_toolkit.history import FileHistory
from struct import pack

console = Console()
session = PromptSession(history=FileHistory("./.history"))

# the commands that the Arduino handles on-board
class usart_commands(Enum):
    RST = 1
    RST_BDM = 2
    CHECK_BDM = 3
    CMD = 4
    GO = 5
    RLA = 6
    DUMP = 7
    STEP = 8
    TRIG_BERR = 9

#NOTE: as of python 3.7, dict must preserve the order that the items were declared in

# commands from the datasheet
bdm_commands = {
    "rreg": 0x2180, # read Data/Address reg - needs to be ORd with the reg num (0-7)
    "wrreg": 0x2080, # read Data/Address reg - needs to be ORd with the reg num
    # the data sheet really says RSREG and WSREG commands are the same (0x2480)
    "rsreg": 0x2580, # read sys reg - needs to be ORd with the reg number
    "wsreg": 0x2480, # write sys reg - needs to be ORd with the sys reg number
    "read": 0x1900, # needs to be ORd with the data width
    "write": 0x1800, # needs to be ORd with the data width
    "dump": 0x1d00, # needs to be ORd with the data width
    "fill": 0x1c00, # needs to be ORd with the data width, not implemented
    "go": 0x0c00,
    "call": 0x0800, # does not always work
    "rst": 0x0400, # resets periphs, NOT the CPU
    "nop": 0x0000
}

#address/data registers
regs = {
    "d0": 0,
    "d1": 1,
    "d2": 2,
    "d3": 3,
    "d4": 4,
    "d5": 5,
    "d6": 6,
    "d7": 7,
    "a0": 8,
    "a1": 9,
    "a2": 10,
    "a3": 11,
    "a4": 12,
    "a5": 13,
    "a6": 14,
    "a7": 15
}

#system registers
sys_regs = {
    "rpc" : 0,
    "pcc" : 1,
    "sr" : 11,
    "usp" : 12,
    "ssp" : 13,
    "sfc" : 14,
    "atemp" : 8,
    "far" : 9,
    "vbr" : 10
}

# some commands require a width specifier; those commands should be OR'd with one of the values below
bdm_sizes = {
    "b": (0 << 6),
    "w": (1 << 6),
    "l": (2 << 6)
}

condition_flags = {
    "c": (1 << 0),
    "v": (1 << 1),
    "z": (1 << 2),
    "n": (1 << 3),
    "x": (1 << 4),
    "s": (1 << 13)
}

# pass it a list of commands/data, each one a short (2 byte)
def send_data(ser, data, print_xfer=False):
    data_len = len(data)
    if data_len == 0:
        print("No command sent")
        return
    ser.write(pack(">BB", usart_commands.CMD.value, data_len * 2))
    rx_bytes = bytes()
    tx_str = []
    rx_str = []
    for d in data:
        ser.write(int.to_bytes(d, length=2))
        tx_str.append(f"${d:04x}")
    # need to bulk read after all the writes otherwise python won't read everything
    for _ in range(data_len):
        b = ser.read(3)
        rx_bytes += b
        rx_str.append(f"${b[2]:1x}{b[0]:02x}{b[1]:02x}")
    if print_xfer:
        tables = []
        table = Table("",show_header=False, style=None)
        table.add_row("TX\nRX")
        tables.append(table)
        for r,t in zip(rx_str, tx_str):
            table = Table("", show_header=False, style=None)
            table.add_row(str(" " + t + '\n' + r))
            tables.append(table)
        panel = Panel(Columns(tables), box=box.SIMPLE)
        console.print(panel)
    return rx_bytes

def print_ser(ser):
    res = ser.readline()
    try:
        s = res.decode().strip()
    except:
        s = res
    console.print(s)

# dump <address to start dumping from> <number of data, in decimal>
# note: the data is dumped 4 bytes at a time; size of dump = number of data * 4
def dump(ser,args):
    if len(args) != 3:
        console.print("Syntax: dump <address to start dumping from in hex, with or without leading \"0x\"> <number of bytes to read (base 10)>")
        console.print("The output filename will be asked for once the command is issued")
        return
    start_addr = int(args[1],16)
    data_len = (int(args[2]) // 4) + (0 if int(args[2]) % 4 == 0 else 1)
    if data_len == 0:
        print("length must be > 0")
        return
    fout_name = input("give output file name: ").strip()
    with open(fout_name, "wb") as fout:
        ser.write(pack(">BI", usart_commands.RLA.value, start_addr))
        b = ser.read(15)
        fout.write(b[9:11] + b[12:14])
        ser.write(pack(">BH", usart_commands.DUMP.value, data_len - 1))
        ser.read(3) # the first DUMP command returns trash
        for _ in track(range(data_len - 1), "Dumping..."):
            b = ser.read(6)
            fout.write(b[0:2] + b[3:5])
       
    

# r(b|w|l)a <address to read from>
def read_address(ser,args):
    if len(args) != 2:
        console.print("Syntax: r(b|w|l)a <address to read from in hex, with or without leading \"0x\">")
        return
    width = args[0][1]
    addr = int(args[1],16)
    upper_addr = (addr >> 16) & 0xffff
    lower_addr = (addr & 0xffff)
    cmd = bdm_commands['read'] | bdm_sizes[width]
    send_data(ser,[cmd, upper_addr, lower_addr])
    if width == 'b' or width == 'w':
        rx_data = send_data(ser,[bdm_commands['nop']])
        outstr = f"0x{upper_addr:04x}{lower_addr:04x} = 0x" + (f"{rx_data[0]:02x}" if width == 'w' else "") + f"{rx_data[1]:02x}"
        if rx_data[2] != 0:
            outstr += " [red](Status: 1)[/red]"
    else:
        rx_data = send_data(ser,[bdm_commands['nop'], bdm_commands['nop']])
        outstr = f"0x{upper_addr:04x}{lower_addr:04x} = 0x{rx_data[0]:02x}{rx_data[1]:02x}{rx_data[3]:02x}{rx_data[4]:02x}"
        if rx_data[2] != 0 or rx_data[5] != 0:
            outstr += " [red](Status: 1)[/red]"
    console.print(outstr)
    return outstr


# w(b|w|l)a <address to write to> <data to write> <another arg silences output to console (optional)>
def write_address(ser,args):
    if len(args) not in range(2,5):
        console.print("Syntax: w(b|w|l)a <address to write to> <data to write in hex, with or without leading \"0x\">")
        return
    # passing another argument suppresses the output to the console
    quiet = (len(args) > 3)
    width = args[0][1]
    addr = int(args[1], 16)
    data = int(args[2], 16)
    cmd = bdm_commands["write"] | bdm_sizes[width]
    if width == 'b' or width == 'w':
        rx_data = send_data(ser,[cmd, (addr >> 16) & 0xffff, addr & 0xffff, data & 0xffff])
    else:
        rx_data = send_data(ser,[cmd, (addr >> 16) & 0xffff, addr & 0xffff, (data >> 16) & 0xffff, data & 0xffff])
    if not quiet:
        console.print(f"Write to 0x{addr:08x} with status {"0" if rx_data[11] == 0 else "1"}") 
        console.print(f"Current value of ", end='')
        read_address(ser,[f"r{width}a)", hex(addr)])


# rreg <register to read>
# returns tuple: (register value, [status bits])
def rreg(ser,args, print_xfer=True):
    if len(args) != 2:
        console.print("Syntax: rreg")
        console.print("Registers can be D0-D7 or A0-A7")
        return
    reg = args[1]
    cmd = bdm_commands['rreg'] | regs[reg]
    rx_data = send_data(ser,[cmd, bdm_commands['nop'], bdm_commands['nop']])
    reg_data = (f"0x{rx_data[3]:02x}{rx_data[4]:02x}{rx_data[6]:02x}{rx_data[7]:02x}",[rx_data[5],[rx_data[8]]])
    if print_xfer:
        console.print(f"{reg} = {regdata2fstr(reg_data)}")
    return reg_data


# wreg <register to write to> <data to write>
def wreg(ser,args):
    if len(args) != 3:
        console.print("Syntax: wreg <register to write to> <data to write in hex, with or without leading \"0x\">")
        console.print("Registers can be D0-D7 or A0-A7")
        return    
    in_reg = args[1]
    cmd = bdm_commands['wrreg'] | regs[in_reg]
    data = int(args[2],16)
    rx_data = send_data(ser,[cmd, (data >> 16) & 0xffff, data & 0xffff])
    console.print(f"Write to {in_reg} with status {"0" if rx_data[8] == 0 else "1"}")
    console.print(f"Current value of {in_reg}: {regdata2fstr(rreg(ser,["rreg",in_reg], False))}")


# rsreg <register to read>
# returns tuple: (register value, [status bits])
def rsreg(ser,args, print_xfer=True):
    if len(args) != 2:
        console.print("Syntax: rsreg <register to read>")
        print_registers(ser,args)
        return
    reg = args[1]
    cmd = bdm_commands['rsreg'] | sys_regs[reg]
    rx_data = send_data(ser,[cmd, bdm_commands['nop'], bdm_commands['nop']])
    reg_data = (f"0x{rx_data[3]:02x}{rx_data[4]:02x}{rx_data[6]:02x}{rx_data[7]:02x}", [rx_data[5],[rx_data[8]]])
    if print_xfer:
        console.print(f"{reg} = {regdata2fstr(reg_data)}")
    return reg_data


# wsreg <reg to write to> <data to write> 
def wsreg(ser,args):
    if len(args) != 3:
        console.print("Syntax: wsreg <register to write to> <data to write in hex, with or without leading \"0x\">")
        print_registers(ser,args)
        return
    in_reg = args[1]
    cmd = bdm_commands['wsreg'] | sys_regs[in_reg]
    data = int(args[2],16)
    rx_data = send_data(ser,[cmd, (data >> 16) & 0xffff, data & 0xffff])
    console.print(f"Write to {in_reg} with status {"0" if rx_data[8] == 0 else "1"}")
    console.print(f"Current value of {in_reg}: {regdata2fstr(rsreg(ser,["rsreg", in_reg], False))}")


# call <address to jump to in hex>, this does not do anything on the device I am using to test this
def call(ser,args):
    console.print("[red]WARNING: This had no effect during testing; results are not guaranteed[/red]")
    addr = int(args[1], 16)
    send_data(ser,[bdm_commands['call'], (addr >> 16) & 0xffff, addr & 0xffff, bdm_commands['nop'], bdm_commands['nop']], print_xfer=True)

def rst(ser,args):
    send_data(ser,[bdm_commands['rst']], print_xfer=True)


# fill <start address in hex> <path to file to send to device>
# I had way too much trouble trying to get the FILL command to work so I am using wla instead
def fill(ser, args):
    if len(args) != 3:
        console.print("Syntax: fill <start address in hex, with or without leading \"0x\"> <path to file to write>")
        return
    addr = int(args[1], 16)
    fin_name = args[2]
    # read the file
    with open(fin_name, "rb") as fin:
        fin_data = fin.read()    
    # pad the data to make it a multiple of 4
    if (len(fin_data) % 4 != 0):
        fin_data += bytearray(4 - len(fin_data) % 4)
    for i in track(range(0,len(fin_data),4)):
        write_address(ser,["wla",hex(addr + i),hex(int.from_bytes(fin_data[i:i+4])),"quiet"])


def print_help(ser, args):
    table = Table("Commands", "Description")
    table.add_row("help/h", "show help")
    table.add_row("dump <addr> <number of bytes>", "dump to file (will prompt for name). The size of the dump will be a multiple of 4.")
    table.add_row("fill <addr> <path to file>", "write the contents of a file to an address in memory. The data will be padded with 0's to make the size a multiple of 4.")
    table.add_row("reset/r", "reset by strobing the RESET line")
    table.add_row("bdm/b - reset to bdm")
    table.add_row("rst", "send the rst command - resets peripherals BUT NOT THE CPU")
    table.add_row("r(b/w/l)a <addr>", "read (byte/word/long) from address (4-byte hex, \"0x\" optional). Ex: rba 00ab0000 to read byte")
    table.add_row("w(b/w/l)a <addr> <data>", "write (byte/word/long) to address (4-byte hex, \"0x\" optional). Ex: rba 00ab0000 deadbeef")
    table.add_row("check/ch/c", "check if system is in BDM mode")
    table.add_row("raw <args>", "send raw bytes, one after another, space delimited, 16-bit hex (\"0x\" optional)")
    table.add_row("nop/n", "no-op")
    table.add_row("go/g", "the arduino will keep sending the \"go\" command until enter is pressed again")
    table.add_row("step/s", "step once")
    table.add_row("call","sends the CALL command (not completely reliable)")
    table.add_row("berr", "send the command to strobe the BERR signal (not reliable)")
    table.add_row("(r/w)reg", "read/write data/address register")
    table.add_row("(r/w)sreg", "read/write system register")
    table.add_row("list", "list system register names")
    table.add_row("readline/rl", "read a line to the console. sometimes pyserial does not print everything in the buffer and this was used for troubleshooting")
    table.add_row("notes", "prints out some notes and potential errors")
    table.add_row("pinout/pins", "display a pinout diagram of the BDM header")
    table.add_row("ir","[underline]i[/underline]nfo [underline]r[/underline]egisters - display register values")
    table.add_row("isr", "[underline]i[/underline]nfo [underline]s[/underline]tatus [underline]r[/underline]egisters")
    table.add_row("iar", "[underline]i[/underline]nfo [underline]a[/underline] [underline]r[/underline]egisters")
    table.add_row("pc", "print the program counter")
    table.add_row("sr", "print status register and the supervisor, X, V, N, C and Z flags")
    table.add_row("# <commands>", "runs console commands, no space is required after \"#\". Ex: \"#ls -l | grep ...\"." \
                  "This will run any command, so don't try rm -rf / --no-preserve-root. Helpful when using the fill command, to search for the desired file.")
    table.add_row("flags", "list the condition flags")
    table.add_row("set/clear <flags>", "set/clear the flags. Multiple flags can be passed without spaces between, ex: \"set zvx\"")
    console.print(table)

def print_registers(ser,args):
    table = Table("Acronym", "Register", title="To use one of the registers with the r/wsreg command, use one of the acronyms below (case insensitive):")
    table.add_row("RPC","Return Program Counter ")
    table.add_row("PCC","Current Instruction Program Counter ")
    table.add_row("SR","Status Register ")
    table.add_row("USP/SSP","(User/Supervisor) Stack Pointer ")
    table.add_row("SFC","Source Function Code Register ")
    table.add_row("DFC","Destination Function Code Register ")
    table.add_row("ATEMP","Temporary Register A ")
    table.add_row("FAR","Fault Address Register ")
    table.add_row("VBR","Vector Base Register ")
    console.print(table)

def print_condition_flags(ser, args):
    table = Table("flag","description")
    table.add_row("x","extend flag - Used in multiple-precision arithmetic operations. In many instructions it is set to the same value as the C bit.")
    table.add_row("n","negative flag - set when the MSB of a result register is set")
    table.add_row("z", "Zero flag - set when all bits of a result register are zero")
    table.add_row("v", "overflow flag - set when two's complement overflow occurs as the result of an operation")
    table.add_row("c", "carry flag - Set when a carry or borrow occurs during an arithmetic operation."\
                  " Also used during shift and rotate instructions to facilitate multiple word operations.")
    table.add_row("s","supervisor flag - 0 when in user mode, 1 when in supervisor mode. you may not be able to chage this flag")
    console.print(table)

def notes(ser, args):
    table = Table("Some things to note", show_lines=True)
    table.add_row("Use \"nop\" to make sure there isn't another response waiting in the BDM buffer")
    table.add_row("\"nop\" can also be used to check if the system isn't responding, it should always respond with $0ffff")
    table.add_row("Use \"readline/rl\" to make sure there isn't another message in the USB buffer")
    table.add_row("The \"step/go\" commands rely on the BDM GO command. In my experience, GO only makes it go several (not one) instructions")
    table.add_row("One source says it makes it go one instruction, another says it refills the pipeline and continues, neither happened in my testing")
    table.add_row("Some functionality could not be tested or did not work - like toggling the BERR pin to trigger a breakpoint")
    table.add_row("This is why this program fully resets the device into BDM mode")
    table.add_row("When spamming GO, the DSCLK signal can theoretically get up to ~4MHz. It's only around 120KHz because the system gets unstable and stops responding")
    table.add_row("The status bit is inconsistent. For example, a write operation might write with status 1 even if the write is successful")
    table.add_row("I am just an intern and I have never used Python this much")
    console.print(table)

# takes register data (return value from rreg or rsreg) and returns a format string for rich to display
def regdata2fstr(reg):
    return f"{reg[0]}{" [red](1)[/red]" if 1 in reg[1] else ""}"


def reset(ser,args):
    ser.write(int.to_bytes(usart_commands.RST.value, length=1))
    print_ser(ser)


def reset_bdm(ser,args):
    ser.write(int.to_bytes(usart_commands.RST_BDM.value, length=1))
    print_ser(ser)


def check_bdm(ser,args):
    ser.write(int.to_bytes(usart_commands.CHECK_BDM.value, length=1))
    res = ser.readline().decode().strip()
    if res.startswith("In"):
        console.print(f"[green]{res}[/green]")
    else:
        console.print(f"[red]{res}[/red]")


def raw(ser,args):
    cmds = [int(s, 16) for s in args[1:]]
    send_data(ser,cmds, print_xfer=True)


def nop(ser,args):
    send_data(ser,[bdm_commands["nop"]], print_xfer=True)


def go(ser,args):
    ser.write(int.to_bytes(usart_commands.GO.value, length=1))
    with console.status("test") as status:
        status.update(status="Spamming go, press enter to stop...", spinner="pong", spinner_style="white")
        input()
    #sending anything tells the arduino to stop
    ser.write(int.to_bytes(usart_commands.GO.value, length=1))
    ser.read(3)
    pc(ser,args)


def step(ser,args):
    ser.write(int.to_bytes(usart_commands.STEP.value, length=1))
    ser.read(3) #don't care about the output
    pc(ser,args)

def pc(ser, args):
    rsreg(ser,["rsreg", "pcc"])

def readline(ser,args):
    print_ser(ser)

def trig_berr(ser,args):
    console.print("[red]WARNING: this had no effect while testing this program; results are not guaranteed[/red]")
    ser.write(int.to_bytes(usart_commands.TRIG_BERR.value, length=1))

def _clear(ser,args):
    if len(args) > 1:
        change_flags(ser,args)
    else:
        console.clear()


# syntax: "set/clear <flags>"
def change_flags(ser,args):
    sr = int(rsreg(ser,["rsreg","sr"],print_xfer=False)[0], 16)
    for c in args[1]:
        if c in condition_flags:
            if args[0] == 'set':
                sr |= condition_flags[c]
            else:
                sr &= ~condition_flags[c]
    wsreg(ser,["wsreg","sr",hex(sr)])
    psr(ser,args)
    

# pinout from https://cmp.felk.cvut.cz/~pisa/m683xx/bdm_driver.html
# this source agrees with other sources
def display_bdm_pinout(ser,args):
    left_pins = Table(show_header=False, show_footer=False, box=box.SIMPLE)
    left_pins.add_row(Align("DS#", "right"))
    left_pins.add_row(Align("GND","right"))
    left_pins.add_row(Align("GND","right"))
    left_pins.add_row(Align("RESET#","right"))
    left_pins.add_row(Align("Vdd","right"))
    right_pins = Table(show_header=False, show_footer=False, box=box.SIMPLE)
    right_pins.add_row(Align("BERR#","left"))
    right_pins.add_row(Align("BKPT#/DSCLK","left"))
    right_pins.add_row(Align("FREEZE","left"))
    right_pins.add_row(Align("IFETCH#/DSI","left"))
    right_pins.add_row(Align("IPIPE#/DSO","left"))
    pins = Table(show_header=False, show_footer=False, box=box.ROUNDED)
    pins.add_row("1 •","• 2")
    pins.add_row("3 •","• 4")
    pins.add_row("5 •","• 6")
    pins.add_row("7 •","• 8")
    pins.add_row("9 •","• 10")
    tables = [left_pins,pins,right_pins]
    panel = Panel(Columns(tables), box=box.SIMPLE)
    console.print(panel)
    
def info_registers(ser, args):
    reg_info = []
    for reg in regs:
        try:
            reg_info.append(rreg(ser, ["rreg",reg], False))
        except:
            data = ("FAILED",[0,0])
            reg_info.append(data)
    table = Table("","0","1","2","3","4","5","6","7", show_lines=True, title="[red](1)[/red] means the read finished with status 1")
    table.add_row("D", regdata2fstr(reg_info[0]), regdata2fstr(reg_info[1]),regdata2fstr(reg_info[2]), regdata2fstr(reg_info[3]),
                  regdata2fstr(reg_info[4]),regdata2fstr(reg_info[5]),regdata2fstr(reg_info[6]), regdata2fstr(reg_info[7]))
    table.add_row("A", regdata2fstr(reg_info[8]), regdata2fstr(reg_info[9]),regdata2fstr(reg_info[10]),regdata2fstr(reg_info[11]),
                  regdata2fstr(reg_info[12]),regdata2fstr(reg_info[13]),regdata2fstr(reg_info[14]), regdata2fstr(reg_info[15]))
    console.print(table)

def info_status_registers(ser,args):
    table = Table("", show_header=False, show_lines=True)
    for reg in sys_regs:
        try:
            table.add_row(reg, regdata2fstr(rsreg(ser, ["rsreg",reg], False)))
        except:
            table.add_row(reg, "FAILED")
    console.print(table)
    

def info_all_registers(ser,args):
    info_registers(ser,args)
    info_status_registers(ser,args)

def psr(ser, args):
    sr = rsreg(ser,["rsreg", "sr"])
    sr_data = int(sr[0],16)
    table = Table("flag","value",show_lines=True)
    table.add_row("S",f"{"[green]1[/green]" if sr_data & condition_flags['s'] != 0 else "[red]0[/red]"}")
    table.add_row("X",f"{"[green]1[/green]" if sr_data & condition_flags['x'] != 0 else "[red]0[/red]"}")
    table.add_row("N",f"{"[green]1[/green]" if sr_data & condition_flags['n'] != 0 else "[red]0[/red]"}")
    table.add_row("Z",f"{"[green]1[/green]" if sr_data & condition_flags['z'] != 0 else "[red]0[/red]"}")
    table.add_row("V",f"{"[green]1[/green]" if sr_data & condition_flags['v'] != 0 else "[red]0[/red]"}")
    table.add_row("C",f"{"[green]1[/green]" if sr_data & condition_flags['c'] != 0 else "[red]0[/red]"}")
    console.print(table)

funcs = {
    "h": print_help, "help": print_help, "list": print_registers, "flags":print_condition_flags,
    "reset": reset, "r": reset, "bdm": reset_bdm, "b": reset_bdm,
    "berr": trig_berr,
    "rst": rst,
    "set": _clear, "clear": _clear, "cls": _clear,
    "check": check_bdm, "ch": check_bdm, "c": check_bdm,
    "raw": raw,
    "rba": read_address, "rwa": read_address, "rla": read_address,
    "wba": write_address, "wwa": write_address, "wla": write_address,
    "rreg": rreg, "wreg": wreg,
    "rsreg": rsreg, "wsreg": wsreg, "pc": pc, "sr": psr,
    "dump": dump, "fill": fill,
    "nop": nop, "n": nop,
    "go": go, "g": go, "step": step, "s": step, "call": call,
    "readline": readline, "rl": readline,
    "notes": notes, "pinout": display_bdm_pinout, "pins": display_bdm_pinout,
    "ir": info_registers, "isr": info_status_registers, "iar": info_all_registers
}


def main():
    if len(sys.argv) < 2:
        console.print("Syntax: bdm_ser.py <serial device>")
        exit()


    console.print("Connecting to serial device...")
    if "test" not in sys.argv:
        ser = serial.Serial(sys.argv[1].strip(), 2000000, timeout=.5)
        while _ := ser.readline().decode().strip() != "Ready": pass
    else:
        ser = None

    console.print("Ready")
    console.print("Type \"h\" or \"help\" to get help")
    console.print("Pressing enter on a blank line will run the previous command")
    console.print("You can run multiple commands on one line by separating them with \"\\\"")

    # initialized in case "enter" is pressed before any args have been passed
    prev_user_in = ""
    stop = False
    while not stop:
        try:
            user_in = session.prompt(">").strip()
            if user_in == "":
                user_in = prev_user_in
            else:
                prev_user_in = user_in

            if user_in == "":
                continue

            for user_commands in user_in.split("\\"):
                args = [s.strip() for s in user_commands.strip().split(" ")]

                # exit/quit here throws a fit and this is a dirty fix for that
                if args[0] == 'q' or args[0] == 'quit':
                    stop = True
                    continue
                
                elif args[0].startswith("#"):
                    user_commands = user_commands.replace("#","")
                    if args[0] == "":
                        console.print(Padding(subprocess.check_output(user_commands, shell=True).decode('utf-8'),pad=(0,0,0,2)))
                    else:
                        console.print(Padding(subprocess.check_output(user_commands, shell=True).decode('utf-8'),pad=(0,0,0,2)))
                    continue

                args[0] = args[0].lower()
                funcs[args[0]](ser, args)
        except KeyError:
            console.print("Invalid command" + f"{'' if user_in == '' else f": {user_in}"}")

        except KeyboardInterrupt:
            stop = True

        except Exception as e:
            console.print(e)

if __name__ == "__main__":
    main()
