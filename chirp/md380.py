# Copyright 2012 Dan Smith <dsmith@danplanet.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.



# This is an incomplete and bug-ridden attempt at a chirp driver for
# the TYT MD-380 by Travis Goodspeed, KK4VCZ.  To use this plugin,
# copy or symlink it into the drivers/ directory of Chirp.
#
# You probably want to read your radio's image with 'md380_dfu.py read
# radio.img' and then open it as a file with chirpw.


from chirp import chirp_common, directory, memmap
from chirp import bitwise, errors
try:
    from chirp.dmr import *
except:
    print("Current Chirp does not have DMR support.")
    #this will fail to import anyway, due to references to things in chirp.dmr


from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueInteger, RadioSettingValueList, \
    RadioSettingValueBoolean, RadioSettingValueString, \
    RadioSettingValueFloat, InvalidValueError, RadioSettings

import logging
LOG = logging.getLogger(__name__)
import tempfile
import os
try:
    from md380tools.md380_dfu import *
except:
    #this will fail to import anyway, due to references to things in md380tools
    print("md380tools not available for import, will not be able to read and write to radio directly")


# Someday I'll figure out Chinese encoding, but for now we'll stick to ASCII.
CHARSET = ["%i" % int(x) for x in range(0, 10)] + \
    [chr(x) for x in range(ord("A"), ord("Z") + 1)] + \
    [" ", ] + \
    [chr(x) for x in range(ord("a"), ord("z") + 1)] + \
    list(".,:;*#_-/&()@!?^ +") + list("\x00" * 100)
DUPLEX = ["", "-", "+", "split"];
#TODO 'DMR' should be added as a valid mode.
MODES = ["DMR", "NFM", "FM"];
TMODES = ["", "Tone", "TSQL"]
BUTTON= {
        "alert_tones_toggle":0x01,
        "emergency_on":0x02,
        "emergency_off":0x03,
        "power_toggle":0x04,
        "monitor":0x05,
        "nuisance_delete":0x06,
        "ota_1":0x07,
        "ota_2":0x08,
        "talkaround":0x0d,
        "scan_toggle":0x0e,
        "squelch_toggle":0x0d,
        "privacy_toggle":0x16,
        "vox_toggle":0x17,
        "zone_select":0x18,
        "manual_dial":0x1e,
        "lone_work_toggle":0x1f,
        }

# Here is where we define the memory map for the radio. Since
# We often just know small bits of it, we can use #seekto to skip
# around as needed.
#
# Large parts of this have yet to be reverse engineered, but I'm
# getting there slowly.
MEM_FORMAT = """

#seekto 0x2180;
struct {
  char messages[288]; // 144 half-length characters, like always.
} messages[50];


#seekto 0x0005F80;
struct {
  ul24 callid;   //DMR Call ID
  u8   flags;    //c1 for group call without an rx tone
                 //c2 for private with no tone
                 //e1 for a group call with an rx tone.
  char name[32]; //U16L chars, of course.
} contacts[1000];


#seekto 0x00018860;
struct {
  char name[32];    //UTF16L, like always.
  u8 flags[10];     //last channel, priority, hold timing, sample time.
  ul16 members[31]; //Just a list; unused entries are zeroed.
} scanlists[20];

#seekto 0x0001EE00;
struct { //0x1F025 into rdt
//unknown features:
    //emergency system
    //private call confirm, emergency alarm ack, data call confirm
    //auto scan
    //lone worker
    //scan list?
    // rx and tx ref freq
    // TOT in seconds
    // TOT rekey delay
    // analog signaling systems, analog ptt id
    //  qt reverse, and reverse burst/turn-off code
    // DCS probably doesn't work at all

  //First byte is 62 for digital, 61 for analog
  u8 mode;   //Upper nybble is 6 for normal squelch, 4 for tight squelch
             //Low nybble is
             //61 for digital, 61 for nbfm, 69 for wbfm
  u8 slot;       //Upper nybble is the color code
                 //lower nybble is bitfield:
                 // |4 for S1, |8 for S2
                 // |2 for RX-ONLY
                 // |1 for talkaround allowed
                 //slotnotes:  0000 0000
                 //            colr 12rt
  char priv;           //Upper nybble is 0 for cleartex, 1 for Basic Privacy, 2 for Enhanced Privacy.
                       //Low nybble is key index.  (E is slot 15, 0 is slot 1.)
  char wase0;          //Unknown, normally E0
                        //0xa0 for "compressed udp data header" turned on
  char power;          //24 for high power, 04 for low power TODO
                        // high nibble:
                         // |8 is color code admit criteria
                         // |4 is cchannel free admit criteria
                         // |2 is high power enabled, off for low power
                         // |1 is vox enabled
                        // low nibble: unknown, usually 0x4
                       // 0 0  0 0   0 0  0 0
                       // CcCf HpVx            

  char wasc3;          //Unknown, normally C3
  ul16 contact;        //Digital contact name.  (TX group.)  TODO
  char unknown[3];     //Certainly analog or digital settings, but I don't know the bits yet.
  u8 scanlist;         //Not yet supported.
  u8 grouplist;        //DMR Group list index. TODO
  char unknown2[3];
  lbcd rxfreq[4];
  lbcd txfreq[4];      //Stored as frequency, not offset.
  lbcd ctone[2];       //Receiver tone.  (0xFFFF when unused.)
  lbcd rtone[2];       //Transmitter tone.
  char yourguess[4];
  char name[32];    //UTF16-LE
} memory[1000];


#seekto 0x1200;
struct {
    char sn[8];
    char model[8];
    char code[16];
    u8 empty[8];
    lbcd prog_yr[2];
    lbcd prog_mon;
    lbcd prog_day;
    u8 empty_10f2c[4];
} info;

#seekto 0x59C0; //0x5be5
//not tested
struct {
    //char enhanced [ 8 ][ 16 ] ; //bitwise doesn't seem to like this
    //this could be better, please fix me
    struct {
        char key[16];
    } enhanced[8];
    u16 basic[ 16 ] ;
} encryption_keys;

#seekto 0x2102; //0x2327 rdt
// not yet tested
struct {
//  for each:
//      0x01 is all alert tones toggle
//      0x02 is emergency on
//      0x03 is emergency off
//      0x04 is high/low power
//      0x05 is monitor
//      0x06 is nuisance delete
//      0x07 is one touch access 1
//      0x08 is ota 2
//      0x09 is ota 3
//      0x0A is ota 4
//      0x0B is ota 5
//      0x0C must be ota 6
//      0x0d is talkaround
//      0x0e is scan toggle
//      0x15 is squelch toggle
//      0x16 is privacy toggle
//      0x17 is vox toggle
//      0x18 is zone select 
//      0x1e is manual dial for private
//      0x1f is lone work toggle

    char button1short;
    char button1long;
    char button2short;
    char button2long;
} buttons;

#seekto 0x20F1; //0x2316 rdt
struct {
// contacts has 8 options
// utilities has 12 options
// call log has 3 options
// scan has 2 options
    char contacts;  // upper:
                    //  |8 is radio disable
                    //  |4 is radio enable
                    //  |2 is remote monitor
                    //  |1 is radio check
                    // lower:
                    //  |8 is 
                    //  |4 is 
                    //  |2 is 
                    //  |1 is
    char utilities1; //when all options available, 0xff
    char utilities2; //when all options available, 0xbf
        // 0x3f if vox disabled, 0xbf if vox enabled
    char utilities3; //when all options on, 0xfb
        // 0xfb if front panel programming allowed, 0xff is fpp disabled
    
} menuoptions;

#seekto 0x149e0;
struct {
  char name[32];    //UTF16-LE
  ul16 members[16]; //16 members for 16 positions on the dial
} bank[99];


#seekto 0x2000;
struct {
    u8 unknownff;
    bbcd prog_yr[2];
    bbcd prog_mon;
    bbcd prog_day;
    bbcd prog_hour;
    bbcd prog_min;
    bbcd prog_sec;
    u8 unknownver[4];       //Probably version numbers.
    u8 unknownff2[52];    //Maybe unused?  All FF.
    char line1[20];         //Top line of text at startup.
    char line2[20];         //Bottom line of text at startup.
    u8 unknownff3[24];      //all FF
    u8 flags1; //FE
    u8 flags2; //6B for no beeps, 6F will all beeps.
    u8 flags3; //EE
    u8 flags4; //FF 
    ul32 dmrid; //0x2084
    u8 flags5[13];  //Unknown settings, seem mostly used.
    u8 screenlit; //00 for infinite delay, 01 for 5s, 02 for 10s, 03 for 15s.
    u8 unknownff4[2];
    u8 unknownzeroes[8];
    u8 unknownff5[16];
    u32 radioname[32]; //Like all other strings.
} general;


#seekto 0x2f003;
u8 selectedzone;

#seekto 0xec20; //offset 0x225 bytes
struct {
    char name[32];          // U16L chars
    ul16 contactidxs[32];    // list of contact indexes in this RX Group
} rxgrouplist[200]; //supposed to be 250, but weirdness in last 12 rxgroups, so now 200

"""

def blankbcd(num):
    """Sets an LBCD value to 0xFFFF"""
    num[0].set_bits(0xFF);
    num[1].set_bits(0xFF);

def utftoasc(utfstring):
    """Converts a UTF16 string to ASCII by dropping the zeroes."""
    toret="";
    for c in utfstring:
        if c!='\x00':
            toret+=c;
    return toret;

def asctoutf(ascstring,size=None):
    """Converts an ASCII string to UTF16."""
    toret="";
    for c in ascstring:
        toret=toret+c+"\x00";
    if size==None: return toret;
    
    #Correct the size here.
    while len(toret)<size:
        toret=toret+"\x00";

    return toret[:size];

class MD380Contact( DMRContact ):
    def __init__(self, *args, **kwargs):
        super( MD380Contact, self).__init__( *args, **kwargs)
        self.name = utftoasc( str( self.name ))
        self.name = self.name.strip()
        if hasattr(self.callid, 'get_value'):
            self.callid = self.callid.get_value()
        if hasattr(self.flags, 'get_value'):
            self.flags = self.flags.get_value()

    def out(self):
        # shouldn't actually modify self.
        self.name = asctoutf( self.name.ljust(16) )
        return super( MD380Contact, self).out()

    def isempty(self):
        if self.name.strip() == '':
            return True

        return False

class MD380ContactList( DMRContactList ):
    def to_csv(self, fh):
        w = csv.DictWriter( fh, fieldnames=self.fieldnames)
        for each in self.cl:
            name = each.name
            callid = each.callid
            flags = each.flags
            if type(callid) == 'instance':
                callid = int(callid, 16)
            if type(flags) == 'instance':
                print("converting!")
                print(flags)
                flags = int(flags, 16)
                print(flags)
            else:
                print("not converting")
            c = {'name':name, 'callid':callid, 'flags':flags}
            w.writerow( c )


class MD380RXGroup( DMRRXGroup ):
    def __init__(self, *args, **kwargs):
        super( MD380RXGroup, self).__init__( *args, **kwargs)
        self.name = utftoasc( str( self.name ))
        self.name = self.name.strip()

    def out(self):
        # shouldn't actually modify self.
        self.name = asctoutf( self.name.ljust(16) )
        return super( MD380RXGroup, self).out()

    def isempty(self):
        if self.name.strip() == '':
            return True

        return False

class MD380Bank(chirp_common.NamedBank):
    """A MD380 Bank"""
    def get_name(self):
        _bank = self._radio._memobj.bank[self.index];
        name = utftoasc(str(_bank.name));
        return name.rstrip();

    def set_name(self, name):
        name = name.upper()
        _bank = self._radio._memobj.bank[self.index];
        _bank.name = asctoutf(name,32);

class MD380BankModel(chirp_common.MTOBankModel):
    """An MD380 Bank model"""
    def get_num_mappings(self):
        return 99
        #return len(self.get_mappings());

    def get_mappings(self):
        banks = []
        for i in range(0, 99):
            #bank = chirp_common.Bank(self, "%i" % (i+1), "MG%i" % (i+1))
            bank = MD380Bank(self, "%i" % (i+1), "MG%i" % (i+1))
            bank._radio=self._radio;
            bank.index = i;
            #print "Bank #%i has name %s" % (i,bank.get_name());
            #if len(bank.get_name())>0:
            banks.append(bank);
        return banks

    def add_memory_to_mapping(self, memory, bank):
        _members = self._radio._memobj.bank[bank.index].members
        #_bank_used = self._radio._memobj.bank_used[bank.index]
        for i in range(0, 16):
            if _members[i] == 0x0000:
                _members[i] = memory.number
                #_bank_used.in_use = 0x0000
                break

    def remove_memory_from_mapping(self, memory, bank):
        _members = self._radio._memobj.bank[bank.index].members

        found = False
        remaining_members = 0
        for i in range(0, len(_members)):
            if _members[i] == (memory.number):
                _members[i] = 0x0000
                found = True
            elif _members[i] != 0x0000:
                remaining_members += 1

        if not found:
            raise Exception("Memory {num} not in " +
                            "bank {bank}".format(num=memory.number,
                                                 bank=bank))
        #if not remaining_members:
        #    _bank_used.in_use = 0x0000

    def get_mapping_memories(self, bank):
        memories = []
        
        _members = self._radio._memobj.bank[bank.index].members
        #_bank_used = self._radio._memobj.bank_used[bank.index]

        #if _bank_used.in_use == 0x0000:
        #    return memories

        for number in _members:
            #Zero items are not memories.
            if number == 0x0000:
                continue
            
            mem=self._radio.get_memory(number)
            print("Appending memory %i" % number)
            memories.append(mem)
        return memories

    def get_memory_mappings(self, memory):
        banks = []
        for bank in self.get_mappings():
            if memory.number in [x.number for x in
                                 self.get_mapping_memories(bank)]:
                banks.append(bank)
        return banks


# Uncomment this to actually register this radio in CHIRP
@directory.register
class MD380Radio(chirp_common.CloneModeRadio, chirp_common.DMRSupport, DMRRadio ):
    """MD380 Binary File"""
    VENDOR = "TYT"
    MODEL = "MD-380"
    FILE_EXTENSION = "img"
    BAUD_RATE = 9600    # This is a lie.
    NO_SERIAL = True

    rxgroup = MD380RXGroup
    contact = MD380Contact
    
    _memsize=262144;

    # def settings(self, **kwargs):
        # if "dmrid" in kwargs.items():
            # k = "dmrid"
            # v = kwargs[k]
            # self._memobj.general.dmrid.set_value(v)
    
# struct {
    # u8 unknownff;
    # bbcd prog_yr[2];
    # bbcd prog_mon;
    # bbcd prog_day;
    # bbcd prog_hour;
    # bbcd prog_min;
    # bbcd prog_sec;
    # u8 unknownver[4];       //Probably version numbers.
    # u8 unknownff2[52];    //Maybe unused?  All FF.
    # char line1[20];         //Top line of text at startup.
    # char line2[20];         //Bottom line of text at startup.
    # u8 unknownff3[24];      //all FF
    # u8 flags1; //FE
    # u8 flags2; //6B for no beeps, 6F will all beeps.
    # u8 flags3; //EE
    # u8 flags4; //FF 
    # ul32 dmrid; //0x2084
    # u8 flags5[13];  //Unknown settings, seem mostly used.
    # u8 screenlit; //00 for infinite delay, 01 for 5s, 02 for 10s, 03 for 15s.
    # u8 unknownff4[2];
    # u8 unknownzeroes[8];
    # u8 unknownff5[16];
    # u32 radioname[32]; //Like all other strings.
# } general;

    @classmethod
    def match_model(cls, filedata, filename):
        return (
               len(filedata) == cls._memsize
            or len(filedata) == cls._memsize+565
            );
    
    def __init__(self,*args, **kwargs):
        super( MD380Radio, self).__init__(*args,**kwargs)

    def fix(self):
        print("MD380.fix(), do not expect to work.")
        # super( MD380Radio, self).fix()
        self.rxgroups = self.rxgrouplist( [ self.rxgroup(x) for x in self._memobj.rxgrouplist ] )
        self.contacts = self.contactlist( [ self.contact(x) for x in self._memobj.contacts    ] )

    def unfix(self):
        print("MD380.unfix(), super broken, don't expect to work!")
        try:
            self.rxgroups.resolve( self )
        except Exception as e:
            print("During rxgroups resolve in unfix()", e)
            raise(e)

        i=0
        for i in range(0, len(self.rxgroups)):
            print(i)
            try:
                g = self.rxgroups[i].out()
                print(g)
                self._memobj.rxgrouplist[i].name = g['name']
                self._memobj.rxgrouplist[i].contactidxs = g['contactidxs']
                i+=1
            except IndexError as e:
                print(e)
        i=0
        for i in range(0, len(self.contacts)):
            print(i)
            try:
                c = self.contacts[i].out()
                print(c)
                self._memobj.contacts[i].name = c['name']
                self._memobj.contacts[i].callid = c['callid']
                self._memobj.contacts[i].flags = c['flags']
                i+=1
            except IndexError as e:
                print(e)


        # super( MD380Radio, self).unfix()
        bm = self.get_bank_model()
        bs = bm.get_mappings()
        for b in bs:
            print(b.get_name())
            ms = bm.get_mapping_memories( b )
            for m in ms:
                bm.remove_memory_from_mapping( m, b )

        l,h = self.get_features().memory_bounds
        for i in xrange(l, h+1):
            m = self.get_memory(i)
            b = bs[(i-1)/16]
            b.set_name( "Z%d"%((i-1)/16) )
            print("adding mem %d to %s"%(i,b.get_name()))
            bm.add_memory_to_mapping(m,b)



            

        print("MD380 unfix")
    
    # Return information about this radio's features, including
    # how many memories it has, what bands it supports, etc
    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_bank = True
        rf.has_bank_index = True
        rf.has_bank_names = True
        rf.can_odd_split = True
        rf.valid_tmodes = TMODES
        rf.memory_bounds = (1, 999)  # Maybe 1000?
        
        rf.valid_bands = [(400000000, 480000000), # 70cm model is most common.
                          (136000000, 174000000)  # 2m model sold separately.
                          ]
        rf.valid_characters = "".join(CHARSET);
        rf.has_settings = True;
        rf.has_tuning_step = False;
        rf.has_ctone=True;
        rf.has_dtcs=False;   #TODO Enable DTCS support.
        rf.has_cross=False;
        rf.valid_modes = list(MODES);
        rf.valid_skips = [""]; #["", "S"]
#        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_tmodes = ["", "Tone", "TSQL"]
        rf.valid_duplexes = list(DUPLEX)
        rf.valid_name_length = 16
        return rf
    
    # Processes the mmap from a file.
    def process_mmap(self):
        if(len(self._mmap)==self._memsize):
            self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)
        elif(len(self._mmap)==self._memsize+565):
            self._memobj = bitwise.parse(MEM_FORMAT, self._mmap[549:])
        self.fix()
        #self._memobj = bitwise.parse(
        #    MEM_FORMAT, self._mmap)
    
    def sync_in(self):
        """Download from the radio."""
        try:
            tmpfile = tempfile.NamedTemporaryFile(delete=False)
            dfu = init_dfu()
            upload_codeplug(dfu, tmpfile.name ) 
            data = tmpfile.read()
            tmpfile.close()
            os.unlink( tmpfile.name )
            self._mmap = memmap.MemoryMap(data)
        except errors.RadioError:
            raise
        except Exception, e:
            raise errors.RadioError("Failed to communicate with radio: %s" % e)
        # if(len(self._mmap)==self._memsize):
            # self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)
        # else:
            # raise errors.RadioError("Incorrect 'Model' selected.")
        self.process_mmap()

    # Do an upload of the radio
    def sync_out(self):
        """Upload to the radio."""
        self.unfix()
        data = self.get_mmap()
        dfu = init_dfu()
        download_codeplug(dfu, data) 

    # Return a raw representation of the memory object, which
    # is very helpful for development
    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number-1])

    # Extract a high-level memory object from the low-level memory map
    # This is called to populate a memory in the UI
    def get_memory(self, number):
        # Get a low-level memory object mapped to the image
        _mem = self._memobj.memory[number-1]
        
        # Create a high-level memory object to return to the UI
        mem = chirp_common.DMRMemory()

        mem.number = number;
        mem.name = utftoasc(str(_mem.name)).rstrip()  # Set the alpha tag
        mem.freq = int(_mem.rxfreq)*10;
        # print("get_memory freq: %d rxfreq: %d"%( mem.freq, _mem.rxfreq))
        
        ctone=int(_mem.ctone)/10.0;
        rtone=int(_mem.rtone)/10.0;

        
        # Anything with an unset frequency is unused.
        # Maybe we should be looking at the mode instead?
        if mem.freq >500e6:
            mem.freq=400e6;
            mem.empty = True;
            mem.name="Empty";
            mem.mode="NFM";
            mem.duplex=""
            mem.offset=mem.freq;
            _mem.mode=0x61; #Narrow FM.
        

        
        #print "Tones for %s are %s and %s" %(
        #    mem.name, rtone, ctone);
        #mem.rtone=91.5
        #mem.ctone=97.4 #88.5
        if ctone==1666.5 and rtone!=1666.5:
            mem.rtone=rtone;
            #mem.ctone=rtone;  #Just one tone here, because the radio can't store a second.
            mem.tmode="Tone";
        elif ctone!=1666.5 and rtone!=1666.5:
            mem.ctone=ctone;
            mem.rtone=rtone;
            mem.tmode="TSQL";
        else:
            mem.tmode="";

        mem.offset = int(_mem.txfreq)*10; #In split mode, offset is the TX freq.
        if mem.offset==mem.freq:
            mem.duplex=""; #Same freq.
            mem.offset=0;
        elif mem.offset==mem.freq+5e6:
            mem.duplex="+";
            mem.offset=5e6;
        elif mem.offset==mem.freq-5e6:
            mem.duplex="-";
            mem.offset=5e6;
        elif mem.offset==mem.freq+6e5:
            mem.duplex="+";
            mem.offset=6e5;
        elif mem.offset==mem.freq-6e5:
            mem.duplex="-";
            mem.offset=6e5;
        else:
            mem.duplex="split";
        
        mem.mode="DMR";
        rmode=_mem.mode&0x0F;
        if rmode==0x02:
            mem.mode="DMR";
        elif rmode==0x01:
            mem.mode="NFM";
        elif rmode==0x09:
            mem.mode="FM";
        else:
            print "WARNING: Mode bytes 0x%02 isn't understood for %s." % (
                _mem.mode, mem.name);

        if mem.mode == "DMR":
            slot = _mem.slot
            print( mem.name, slot )
            # print(mem.name)
            # print("\tSlot: ",bin(slot))
            color = slot >> 4
            rxonly = slot & 0x2
            slot = (slot & 12) >> 2
            print("C,S,rxonly", color, slot, rxonly)
            mem.timeslot = slot
            mem.colorcode = color
        mem.txgroup = _mem.contact
        mem.rxgroup = _mem.grouplist
        if mem.name in ["Empty",'']:
            mem.empty = True

        
        return mem

    # Store details about a high-level memory to the memory map
    # This is called when a user edits a memory in the UI
    def set_memory(self, mem):
        try:
            mem.resolve( self )
        except AttributeError as e:
            pass #print("non-dmr memory",e)
        # Get a low-level memory object mapped to the image
        _mem = self._memobj.memory[mem.number-1]

        # Convert to low-level frequency representation
        _mem.rxfreq = mem.freq/10;
        # print("set_memory rxfreq: %d freq: %d"%( _mem.rxfreq, mem.freq))
        
        # Janky offset support.
        # TODO Emulate modes other than split.
        if mem.duplex=="split":
            _mem.txfreq = mem.offset/10;
        elif mem.duplex=="+":
            _mem.txfreq = mem.freq/10+mem.offset/10;
        elif mem.duplex=="-":
            _mem.txfreq = mem.freq/10-mem.offset/10;
        else:
            _mem.txfreq = _mem.rxfreq;
        _mem.name = asctoutf(mem.name,32);
        
        #print "Tones in mode %s of %s and %s for %s" % (
        #    mem.tmode, mem.ctone, mem.rtone, mem.name);
        # These need to be 16665 when unused.
        _mem.ctone=mem.ctone*10;
        _mem.rtone=mem.rtone*10;
        
        if mem.tmode=="Tone":
            blankbcd(_mem.ctone); #No receiving tone.
        elif mem.tmode=="TSQL":
            pass;
        else:
            blankbcd(_mem.ctone);
            blankbcd(_mem.rtone);
        
        if mem.mode=="FM":
            _mem.mode=0x69;
        elif mem.mode=="NFM":
            _mem.mode=0x61;
        elif mem.mode=="DMR":
            _mem.mode=0x62;
        else:
            _mem.mode=0x69;
        
        if _mem.slot==0xff:
            _mem.slot=0x14;  #TODO Make this 0x18 for S2.

        if mem.mode == "DMR":
            _mem.slot = mem.colorcode << 4 | mem.timeslot << 2 
            _mem.contact = mem.txgroup
            _mem.grouplist = mem.rxgroup
            print("Setting mem %s txgroup to contact idx %s slot= %x"%(str(mem.number), str(mem.txgroup), _mem.slot))

        if mem.empty:
            _mem.mode = 0xff
            _mem.slot = 0xff
            _mem.contact = 0xffff
            _mem.txfreq = 166666665
            _mem.rxfreq = 166666665
            pass #how does radio determine if a channel is empty?

            


    def get_settings(self):
        _general = self._memobj.general
        _info = self._memobj.info
        
        basic = RadioSettingGroup("basic", "Basic")
        info = RadioSettingGroup("info", "Model Info")
        general = RadioSettingGroup("general", "General Settings");
        
        
        #top = RadioSettings(identity, basic)
        top = RadioSettings(general)
        general.append(RadioSetting(
                "dmrid", "DMR Radio ID",
                RadioSettingValueInteger(0, 100000000, _general.dmrid)));
        general.append(RadioSetting(
                "line1", "Startup Line 1",
                RadioSettingValueString(0, 10, utftoasc(str(_general.line1)))));
        general.append(RadioSetting(
                "line2", "Startup Line 2",
                RadioSettingValueString(0, 10, utftoasc(str(_general.line2)))));
        return top

    def set_settings(self, settings):
        _general = self._memobj.general
        _info = self._memobj.info
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            if not element.changed():
                continue
            try:
                setting = element.get_name()
                #oldval = getattr(_settings, setting)
                newval = element.value
                
                #LOG.debug("Setting %s(%s) <= %s" % (setting, oldval, newval))
                if setting=="line1":
                    _general.line1=asctoutf(str(newval),20);
                elif setting=="line2":
                    _general.line2=asctoutf(str(newval),20);
                else:
                    print("Setting %s <= %s" % (setting, newval))
                    setattr(_general, setting, newval)
            except Exception, e:
                LOG.debug(element.get_name())
                raise

    def get_bank_model(self):
        return MD380BankModel(self)
