unit uHCI;

{$mode objfpc}{$H+}
// { $define show_data}

interface

uses
  Classes, SysUtils, Serial, BCM2710, GlobalConst, uLog;
(*

Connection to BCM4343 chip

UART0      115200 BAUD, 8 DATA, 1 STOP, NO PARITY, NO FLOW CONTROL

PIN 15     SET TO INPUT
PIN 32     SET TO ALT3
PIN 33     SET TO ALT3

OCF        OPCODE GROUP FIELD (HIGHER 6 BITS OF OP CODE)

  OGF_LINK_CONTROL	   0x01 COMMANDS TO CONTROL CONNECTIONS TO OTHER BT DEVICES
  OGF_LINK_POLICY	     0x02
  OGF_HOST_CONTROL 	   0x03 COMMANDS TO ACCESS AND CONTROL HARDWARE
  OGF_INFORMATIONAL	   0x04 COMMANDS THAT PROVIDE INFO ON AND CAPABLILITIES OF HARDWARE
  OGF_STATUS           0x05 COMMANDS THAT ACCESS STATE OF LINK MANAGER
  OGF_LE_CONTROL       0x08 COMMANDS THAT ACCESS THE LOW ENEGY (LE) FEATURES
  OGF_VENDOR           0x3F VENDOR SPECIFIC COMMANDS

  OP CODE = (OGF SHL 10) OR OCF

COMMAND FORMAT

    HCI_COMMAND_PKT    0x01
    LO (OP CODE)
    HI (OP CODE)
    PARAM LENGTH
    PARAMS....
*)

const

  HCI_COMMAND_PKT		            = $01;
  HCI_ACLDATA_PKT		            = $02;
  HCI_SCODATA_PKT		            = $03;
  HCI_EVENT_PKT		              = $04;
  HCI_VENDOR_PKT		            = $ff;

  // Markers
  FIRMWARE_START                = 100;
  FIRMWARE_END                  = 101;
  DELAY_50MSEC                  = 102;
  DELAY_2SEC                    = 103;
  INIT_COMPLETE                 = 104;
  FLUSH_PORT                    = 105;
  OPEN_PORT                     = 106;
  CLOSE_PORT                    = 107;

  // from bluez-5.32 hciattach_bcm43xx.c
  CC_MIN_SIZE                   = 7;

  // Link Layer specification Section 4.4.3, Core 4.1 page 2535
  LL_SCAN_WINDOW_MAX		        = 10240000;	// 10.24s
  LL_SCAN_INTERVAL_MAX	        = 10240000;	// 10.24s

  SCAN_WINDOW		                =	200000;
  SCAN_INTERVAL		              =	500000;
  SCAN_DURATION		              =	10000000;	// 10s

  BDADDR_LEN			              = 6;

  BDADDR_TYPE_PUBLIC		        = 0;
  BDADDR_TYPE_RANDOM		        = 1;

  BT_MAX_HCI_EVENT_SIZE	        = 257;
  BT_MAX_HCI_COMMAND_SIZE	      = 258;
  BT_MAX_DATA_SIZE	            = BT_MAX_HCI_COMMAND_SIZE;

  BT_BD_ADDR_SIZE		            = 6;
  BT_CLASS_SIZE		              = 3;
  BT_NAME_SIZE		              = 248;

  OGF_MARKER                    = $00;
  OGF_LINK_CONTROL	            =	$01;
  OGF_LINK_POLICY	              =	$02;
  OGF_HOST_CONTROL 	            = $03;
  OGF_INFORMATIONAL	            =	$04;
  OGF_LE_CONTROL                = $08;
  OGF_VENDOR                    = $3f;

  //OP_CODE_WRITE_CLASS_OF_DEVICE	= OGF_HCI_CONTROL_BASEBAND or $024;

  INQUIRY_LAP_GIAC		          = $9E8B33;	// General Inquiry Access Code

  INQUIRY_LENGTH_MIN		        = $01;		// 1.28s
  INQUIRY_LENGTH_MAX		        = $30;		// 61.44s
  //#define INQUIRY_LENGTH(secs)		(((secs) * 100 + 64) / 128)
  INQUIRY_NUM_RESPONSES_UNLIMITED	= $00;


type
  TBTMarkerEvent = procedure (no : integer);
  TBTLEEvent = procedure (SubEvent : byte; Params : array of byte);

  TBDAddr = array [0 .. BDADDR_LEN - 1] of byte;
  TLEData = array [0 .. 30] of byte;
  TLEKey = array [0 .. 15] of byte;

  PQueueItem = ^TQueueItem;
  TQueueItem = record
    OpCode : Word;
    Params : array of byte;
    Prev, Next : PQueueItem;
  end;

function  OpenUART0 : boolean;
procedure CloseUART0;

procedure AddHCICommand (OGF : byte; OCF : Word; Params : array of byte); overload;
procedure AddHCICommand (OpCode : Word; Params : array of byte); overload;

function ogf (Op : Word) : byte;
function ocf (Op : Word) : Word;
function BDAddrToStr (Addr : TBDAddr) : string;
function ErrToStr (Code : byte) : string;

procedure NoOP;
procedure AddMarker (Marker : Word);
procedure SetMarkerEvent (anEvent : TBTMarkerEvent);
procedure SetLEEvent (anEvent : TBTLEEvent);

// HCI Commands
procedure ResetChip;
procedure ReadLocalName;
procedure ReadScanEnable;
procedure WriteScanEnable (Enable : byte);

// Link Control
procedure Inquiry;

// Informational Parameters
procedure ReadLocalVersion;
procedure ReadLocalSupportedCommands;
procedure ReadLocalSupportedFeatures;
procedure ReadBDADDR;

// LE
procedure SetLEEventMask (Mask : QWord);
procedure ReadLEBufferSize;
procedure ReadLESupportedFeatures;
procedure SetLERandomAddress (Addr : TBDAddr);
procedure SetLEAdvertisingParameters (MinInterval, MaxInterval : Word;
                                     Type_ : byte;
                                     OwnAddressType, PeerAddressType : byte;
                                     PeerAddr : TBDAddr;
                                     ChannelMap, FilterPolicy : byte);
procedure ReadLEAdvertisingChannelTxPower;
procedure SetLEAdvertisingData (Data : array of byte);
procedure SetLEScanResponseData (Data : array of byte);
procedure SetLEAdvertisingEnable (State : boolean);
procedure SetLEScanParameters (Type_ : byte; Interval, Window : Word;
                               OwnAddressType, FilterPolicy : byte);
procedure SetLEScanEnable (State, Duplicates : boolean);
procedure LERand;
procedure StartLEEncryption (Handle : Word; Rand : QWord; Diversifier : Word; Key : TLEKey);

// BCM Vendor Specific
procedure BCMSetBDAddr (Addr : TBDAddr);
procedure BCMEnableRadio (Enable : boolean);
procedure BCMLoadFirmware (fn : string);

var
  ChipName : string = '';
  Ver : byte = 0;
  Rev : Word = 0;
  BDAddr : TBDAddr = ($b8, $27, $e8, $cc, $72, $27);
  FWHandle : integer; // firmware file handle

implementation

uses Platform, GlobalTypes, GlobalConfig, Threads, FileSystem, SyncObjs;

var
  UART0 : PSerialDevice = nil;
  First : PQueueItem = nil;
  Last : PQueueItem = nil;
  ReadHandle : TThreadHandle = INVALID_HANDLE_VALUE;
  Queue : TMailslotHandle;
  QueueHandle : TThreadHandle = INVALID_HANDLE_VALUE;
  QueueEvent : TEvent;
  RxBuffer : array of byte;
  MarkerEvent : TBTMarkerEvent = nil;
  LEEvent : TBTLEEvent = nil;

const
  ADV_IND                     = $00; // Connectable undirected advertising (default)
  ADV_DIRECT_IND_HI           = $01; // Connectable high duty cycle directed advertising
  ADV_SCAN_IND                = $02; // Scannable undirected advertising
  ADV_NONCONN_IND             = $03; // Non connectable undirected advertising
  ADV_DIRECT_IND_LO           = $04; // Connectable low duty cycle directed advertising


function EventTypeToStr (Type_ : byte) : string;
begin
  case Type_ of
    ADV_IND           : Result := 'Connectable undirected advertising (default)';
    ADV_DIRECT_IND_HI : Result := 'Connectable high duty cycle directed advertising';
    ADV_SCAN_IND      : Result := 'Scannable undirected advertising';
    ADV_NONCONN_IND   : Result := 'Non connectable undirected advertising';
    ADV_DIRECT_IND_LO : Result := 'Connectable low duty cycle directed advertising';
    else                Result := 'Reserved for future use (' + Type_.ToHexString(2) + ')';
  end;
end;

function ogf (op : Word) : byte;
begin
  Result := (op shr 10) and $3f;
end;

function ocf (op : Word) : Word;
begin
  Result := op and $3ff;
end;

procedure SetMarkerEvent (anEvent : TBTMarkerEvent);
begin
  MarkerEvent := anEvent;
end;

procedure SetLEEvent (anEvent : TBTLEEvent);
begin
  LEEvent := anEvent;
end;

function BDAddrToStr (Addr : TBDAddr) : string;
var
  i : integer;
begin
  Result := '';
  for i := 0 to 5 do
    if i = 0 then
      Result := Addr[i].ToHexString(2)
    else
      Result := Result + ':' + Addr[i].ToHexString(2);
end;

procedure DecodeEvent (ev : array of byte);
var
  len, num : byte;
  op : Word;
  nr, ofs, se : byte; // number of responses
  i, j : integer;
  prm : array of byte;
  s : string;

  procedure ListAD (ad : array of byte);
  begin
    if length (ad) = 0 then exit;
    case ad[0] of
      $01 : // flags
        if length (ad) = 2 then
          Log ('Flags : ' + ad[1].ToHexString (2));
      $ff : // manufacturer specific
        begin
          Log ('Manufacturer Specific');
        end;
      end;
  end;

begin
  if length (ev) < 3 then exit;
  if ev[0] <> HCI_EVENT_PKT then exit;
  len := ev[2];
  num := 0;
  if len + 2 <> high (ev) then exit;
  case ev[1] of             // event code
    $01 :   // inquiry complete
      begin
        if len = 1 then Log ('Inquiry Complete : ' + ErrToStr (ev[3]));
      end;
    $02 : // inquiry result event
      begin
        if len > 1 then
           begin
             nr := ev[3];
             if (nr * 14) + 1 = len then
                begin
                  Log ('Inquiry result nos ' + ev[3].tostring + ' Len ' + len.ToString);
                  for i := 1 to nr do
                    begin
                      ofs := ((i - 1) * 14) + 3;
                      Log ('  Controller ' + i.ToString);
                      s := '  BD';
                      for j := 0 to 5 do s := s + ' ' + ev[j + ofs].ToHexString(2);
                      Log (s);
                    end;
                end;
           end;
      end;
    $0e :   // command complete
      begin
        num := ev[3];          // num packets controller can accept
        op := ev[5] * $100 + ev[4];
        //Log ('OGF ' + inttohex (ogf (op), 2) + ' OCF ' + inttohex (ocf (op), 3) + ' OP Code ' + inttohex (op, 4) + ' Num ' + num.ToString + ' Len ' + len.ToString);
        if (len > 3) and (ev[6] > 0) then Log ('Status ' + ErrToStr (ev[6]));
        case op of
          $0c14 : // read name
            begin
              ChipName := '';
              i := 7;
              while (i <= len + 3) and (ev[i] <> $00) do
                begin
                  ChipName := ChipName + chr (ev[i]);
                  i := i + 1;
                end;
            end;
          $1001 : // read local version
            begin
              if len = 12 then
                begin
                  Ver := ev[7];
                  Rev := ev[9] * $100 + ev[8];
                end;
            end;
          $1009 : // read bd addr
            begin
              if len = 10 then for i := 0 to 5 do BDAddr[i] := ev[7 + i];
            end;
          $2007 : // read le channel tx power
            begin
              if len = 5 then Log ('Tx Power ' + ev[7].ToString);
            end;
          end;  // case op
        end;  // command complete
      $0f : // command status
        begin
      //    Log ('Command Status');
          if (len = 4) then
            begin
              num := ev[4];
              op := ev[6] * $100 + ev[5];
              Log ('  Status ' + inttohex (ev[3], 2));
              Log ('  OGF ' + inttohex (ogf (op), 2) + ' OCF ' + inttohex (ocf (op), 3) + ' OP Code ' + inttohex (op, 4));
            end;
        end;
      $3e : // le meta event
        begin
          if (len > 2) then
            begin
              se := ev[3];
              if Assigned (LEEvent) then
                begin
                  SetLength (prm, len - 1);
                  Move (ev[4], prm[0], len - 1);
                  LEEvent (se, prm);
                end;
            end; // len <=
        end   // case
      else Log ('Unknown command ' + ev[1].ToString);
     end;
  if num > 0 then QueueEvent.SetEvent;
end;

function ReadExecute (Parameter : Pointer) : PtrInt;
var
{$ifdef show_data}
  s : string;
{$endif}
  c : LongWord;
  b : byte;
  i, j, rm : integer;
  decoding : boolean;
  pkt : array of byte;
  res : LongWord;
begin
  Result := 0;
  c := 0;
  while True do
    begin
      res := SerialDeviceRead (UART0, @b, 1, SERIAL_READ_NONE, c);
      if (res = ERROR_SUCCESS) and (c = 1) then
        begin             // One byte was received, try to read everything that is available
          SetLength (RxBuffer, length (RxBuffer) + 1);
          RxBuffer[high (RxBuffer)] := b;
    (*      res := SerialDeviceRead (UART0, @b, 1, SERIAL_READ_NON_BLOCK, c);
          while (res = ERROR_SUCCESS) and (c = 1) do
            begin
              SetLength (RxBuffer, length (RxBuffer) + 1);
              RxBuffer[high (RxBuffer)] := b;
              res := SerialDeviceRead (UART0, @b, 1, SERIAL_READ_NON_BLOCK, c);
            end;      *)
          i := 0;
          decoding := true;
          while decoding do
            begin
              decoding := false;
              if (i + 2 <= high (RxBuffer)) then // mimumum
                 if i + RxBuffer[i + 2] + 2 <= high (RxBuffer) then
                   begin
                     SetLength (pkt, RxBuffer[i + 2] + 3);
                     for j := 0 to length (pkt) - 1 do pkt[j] := RxBuffer[i + j];
{$ifdef show_data}
                     s := '';
                     for j := low (pkt) to high (pkt) do s := s + ' ' + pkt[j].ToHexString (2);
                     Log ('<--' + s);
{$endif}
                     DecodeEvent (pkt);
                     i := i + length (pkt);
                     decoding := i < high (RxBuffer);
                   end;
            end; // decoding
          if i > 0 then
            begin
              rm := length (RxBuffer) - i;
//              Log ('Remaining ' + IntToStr (rm));
              if rm > 0 then
                for j := 0 to rm - 1 do RxBuffer[j] := RxBuffer[j + i];
              SetLength (RxBuffer, rm);
            end;
        end;
    end;
end;

function OpenUART0 : boolean;
var
  res : LongWord;
  BoardType: LongWord;
  FlowControl: LongWord;
begin
  Result := false;
  UART0 := SerialDeviceFindByDescription (BCM2710_UART0_DESCRIPTION);
  if UART0 = nil then
      begin
        Log ('Can''t find UART0');
        exit;
      end;

  BoardType := BoardGetType;
  FlowControl := SERIAL_FLOW_NONE;
  case BoardType of
    BOARD_TYPE_RPI_ZERO_W,
    BOARD_TYPE_RPI3B_PLUS,
    BOARD_TYPE_RPI3A_PLUS: FlowControl := SERIAL_FLOW_RTS_CTS;
    //BOARD_TYPE_RPI4B
    //BOARD_TYPE_RPI400
    //BOARD_TYPE_RPI_COMPUTE4
    //BOARD_TYPE_RPI_ZERO2_W
  end;

  res := SerialDeviceOpen (UART0, 115200, SERIAL_DATA_8BIT, SERIAL_STOP_1BIT, SERIAL_PARITY_NONE, FlowControl, 0, 0);
  if res = ERROR_SUCCESS then
    begin
      GPIOFunctionSelect (GPIO_PIN_14, GPIO_FUNCTION_IN);
      GPIOFunctionSelect (GPIO_PIN_15, GPIO_FUNCTION_IN);

      GPIOFunctionSelect (GPIO_PIN_32, GPIO_FUNCTION_ALT3);     // TXD0
      GPIOFunctionSelect (GPIO_PIN_33, GPIO_FUNCTION_ALT3);     // RXD0
      GPIOPullSelect (GPIO_PIN_32, GPIO_PULL_NONE);
      GPIOPullSelect (GPIO_PIN_33, GPIO_PULL_UP);

      if FlowControl > SERIAL_FLOW_NONE then
      begin
        GPIOFunctionSelect (GPIO_PIN_16, GPIO_FUNCTION_IN);
        GPIOFunctionSelect (GPIO_PIN_17, GPIO_FUNCTION_IN);

        GPIOFunctionSelect(GPIO_PIN_30,GPIO_FUNCTION_ALT3);     // RTS0
        GPIOFunctionSelect(GPIO_PIN_31,GPIO_FUNCTION_ALT3);     // CTS0
        GPIOPullSelect(GPIO_PIN_30,GPIO_PULL_UP);
        GPIOPullSelect(GPIO_PIN_31,GPIO_PULL_NONE);
      end;

      Result := true;
      ReadHandle := BeginThread (@ReadExecute, nil, ReadHandle, THREAD_STACK_DEFAULT_SIZE);
      Result := ReadHandle <> INVALID_HANDLE_VALUE;
    end;
end;

procedure CloseUART0;
begin
  if ReadHandle <> INVALID_HANDLE_VALUE then KillThread (ReadHandle);
  ReadHandle := INVALID_HANDLE_VALUE;
  if UART0 <> nil then SerialDeviceClose (UART0);
  UART0 := nil;
end;

procedure AddHCICommand (OGF : byte; OCF : Word; Params : array of byte);
begin
  AddHCICommand ((OGF shl 10) or OCF, Params);
end;

procedure AddHCICommand (OpCode : Word; Params : array of byte);
var
  anItem : PQueueItem;
  i : integer;
begin
  New (anItem);
  anItem^.OpCode := OpCode;
  SetLength (anItem^.Params, length (Params));
  for i := 0 to length (Params) - 1 do anItem^.Params[i] := Params[i];
  anItem^.Next := nil;
  anItem^.Prev := Last;
  if First = nil then First := anItem;
  if Last <> nil then Last^.Next := anItem;
  Last := anItem;
  if MailSlotSend (Queue, Integer (anItem)) <> ERROR_SUCCESS then
    Log ('Error adding Command to queue.');
end;

function QueueHandler (Parameter : Pointer) : PtrInt;
var
  anItem : PQueueItem;
  Cmd : array of byte;
  i : integer;
  res, count : LongWord;
{$ifdef show_data}
  s : string;
{$endif}
begin
  Result := 0;
  while true do
    begin
      QueueEvent.ResetEvent;
      anItem := PQueueItem (MailslotReceive (Queue));
      if anItem <> nil then
        begin
          if (ogf (anItem^.OpCode) = OGF_MARKER) and (ocf (anItem^.OpCode) > 0) then
            begin
              case ocf (anItem^.OpCode) of
                DELAY_50MSEC : QueueEvent.WaitFor (50);
                DELAY_2SEC   : QueueEvent.WaitFor (2000);
                OPEN_PORT    : OpenUART0;
                CLOSE_PORT   : CloseUART0;
                end;
              if Assigned (@MarkerEvent) then MarkerEvent (ocf (anItem^.OpCode));
            end
          else
            begin
              SetLength (Cmd, length (anItem^.Params) + 4);
              Cmd[0] := HCI_COMMAND_PKT;
              Cmd[1] := lo (anItem^.OpCode);          // little endian so lowest sent first
              Cmd[2] := hi (anItem^.OpCode);
              Cmd[3] := length (anItem^.Params);
              for i := 0 to length (anItem^.Params) - 1 do Cmd[4 + i] := anItem^.Params[i];
              count := 0;
{$ifdef show_data}
              s := '';
              for i := 0 to length (Cmd) - 1 do s := s + ' ' + Cmd[i].ToHexString (2);
              Log ('--> ' + s);
{$endif}
              res := SerialDeviceWrite (UART0, @Cmd[0], length (Cmd), SERIAL_WRITE_NONE, count);
              if res = ERROR_SUCCESS then
                begin
                  if QueueEvent.WaitFor (10000) <> wrSignaled then
                    Log ('Timeout waiting for BT Response.'); // should send nop ???
                end
              else
                Log ('Error writing to BT.');
            end;
          SetLength (anItem^.Params, 0);
          Dispose (anItem);
        end;
    end;
end;

procedure NoOP;  // in spec but not liked by BCM chip
begin
  AddHCICommand ($00, $00, []);
end;

procedure AddMarker (Marker : Word);
begin
  AddHCICommand (OGF_MARKER, Marker and $3ff, []);
end;

// host control
procedure ResetChip;
begin
  AddHCICommand (OGF_HOST_CONTROL, $03, []);
end;

procedure ReadLocalName;
begin
  AddHCICommand (OGF_HOST_CONTROL, $14, []);
end;

procedure ReadScanEnable;
begin
  AddHCICommand (OGF_HOST_CONTROL, $19, []);
end;

procedure WriteScanEnable (Enable : byte);
begin
  AddHCICommand (OGF_HOST_CONTROL, $14, [Enable]);
end;

// link control
procedure Inquiry;
var
  Params : array of byte;
begin
  SetLength (Params, 5);
  Params[0] := $33;
  Params[1] := $8b;
  Params[2] := $9e;
  Params[3] := 1;       // inquiry length x * 1.28 secs
  Params[4] := $00;     // unlimited number of responses
  AddHCICommand (OGF_LINK_CONTROL, $01, Params);
end;

// informational parameters
procedure ReadLocalVersion;
begin
  AddHCICommand (OGF_INFORMATIONAL, $01, []);
end;

procedure ReadLocalSupportedCommands;
begin
  AddHCICommand (OGF_INFORMATIONAL, $02, []);
end;

procedure ReadLocalSupportedFeatures;
begin
  AddHCICommand (OGF_INFORMATIONAL, $03, []);
end;

procedure ReadBDADDR;
begin
  AddHCICommand (OGF_INFORMATIONAL, $09, []);
end;

// le control
procedure SetLEEventMask (Mask : QWord);
var
  Params : array of byte;
  MaskHi, MaskLo : DWord;
begin
  MaskHi := hi (Mask);
  MaskLo := lo (Mask);
  SetLength (Params, 8);
  Params[0] := MaskLo and $ff;   // lsb
  Params[1] := (MaskLo shr 8) and $ff;
  Params[2] := (MaskLo shr 16) and $ff;
  Params[3] := (MaskLo shr 24) and $ff;
  Params[4] := MaskHi and $ff;   // lsb
  Params[5] := (MaskHi shr 8) and $ff;
  Params[6] := (MaskHi shr 16) and $ff;
  Params[7] := (MaskHi shr 24) and $ff;
  AddHCICommand (OGF_LE_CONTROL, $01, Params);
end;

procedure ReadLEBufferSize;
begin
  AddHCICommand (OGF_LE_CONTROL, $02, []);
end;

procedure ReadLESupportedFeatures;
begin
  AddHCICommand (OGF_LE_CONTROL, $03, []);
end;

procedure SetLERandomAddress (Addr : TBDAddr);
begin
  AddHCICommand (OGF_LE_CONTROL, $05, [Addr[5], Addr[4], Addr[3], Addr[2], Addr[1], Addr[0]]);
end;

procedure SetLEAdvertisingParameters (MinInterval, MaxInterval : Word;
                                     Type_ : byte;
                                     OwnAddressType, PeerAddressType : byte;
                                     PeerAddr : TBDAddr;
                                     ChannelMap, FilterPolicy : byte);
begin
  AddHCICommand (OGF_LE_CONTROL, $06, [lo (MinInterval), hi (MinInterval),
                                       lo (MaxInterval), hi (MaxInterval),
                                       Type_, OwnAddressType, PeerAddressType,
                                       PeerAddr[0], PeerAddr[1], PeerAddr[2],
                                       PeerAddr[3], PeerAddr[4], PeerAddr[5],
                                       ChannelMap, FilterPolicy]);
end;

procedure ReadLEAdvertisingChannelTxPower;
begin
  AddHCICommand (OGF_LE_CONTROL, $07, []);
end;

procedure SetLEAdvertisingData (Data : array of byte);
var
  Params : array of byte;
  Len : byte;
  i : integer;
begin
   SetLength (Params, 32);
   for i := 1 to 31 do Params[i] := 0;       // clear data
   Len := length (Data);
   if Len > 31 then Len := 31;
   Params[0] := Len;
   for i := 0 to Len - 1 do Params[i + 1] := Data[i];
   AddHCICommand (OGF_LE_CONTROL, $08, Params);
end;

procedure SetLEScanResponseData (Data : array of byte);
var
  Params : array of byte;
  Len : byte;
  i : integer;
begin
   SetLength (Params, 32);
   for i := 1 to 31 do Params[i] := 0;       // clear data
   Len := length (Data);
   if Len > 31 then Len := 31;
   Params[0] := Len;
   for i := 0 to Len - 1 do Params[i + 1] := Data[i];
   AddHCICommand (OGF_LE_CONTROL, $09, Params);
end;

procedure SetLEAdvertisingEnable (State : boolean);
begin
  if State then
    AddHCICommand (OGF_LE_CONTROL, $0a, [$01])
  else
    AddHCICommand (OGF_LE_CONTROL, $0a, [$00]);
end;

procedure SetLEScanParameters (Type_ : byte; Interval, Window : Word;
                               OwnAddressType, FilterPolicy : byte);
begin
  AddHCICommand (OGF_LE_CONTROL, $0b, [Type_, lo (Interval), hi (Interval),
                                       lo (Window), hi (Window),
                                       OwnAddressType, FilterPolicy]);
end;

procedure SetLEScanEnable (State, Duplicates : boolean);
var
  Params : array of byte;
begin
  SetLength (Params, 2);
  if State then Params[0] := $01 else Params[0] := $00;
  if Duplicates then Params[1] := $01 else Params[1] := $00;
  AddHCICommand (OGF_LE_CONTROL, $0c, Params);
end;

procedure LERand;
begin
  AddHCICommand (OGF_LE_CONTROL, $18, []);
end;

procedure StartLEEncryption (Handle : Word; Rand : QWord; Diversifier : Word; Key : TLEKey);
begin
  // todo
end;

// BCM vendor specific      http://www.cypress.com/file/298311/download
procedure BCMSetBDAddr (Addr : TBDAddr);
var
  i : integer;
  Params : array of byte;
begin
  SetLength (Params, 6);
  for i := 0 to 5 do Params[i] := Addr[i];
  AddHCICommand (OGF_VENDOR, $001, Params);
end;

procedure BCMLoadFirmware (fn : string);
var
  hdr : array [0 .. 2] of byte;
  Params : array of byte;
  n, len : integer;
  Op : Word;
begin               // firmware file BCM43430A1.hcd under \lib\firmware
//  Log ('Loading Firmware file ' + fn);
  FWHandle := FSFileOpen (fn, fmOpenRead);
  if FWHandle > 0 then
    begin
      AddMarker (FIRMWARE_START);
      AddHCICommand (OGF_VENDOR, $2e, []);
      AddMarker (DELAY_50MSEC);
      n := FSFileRead (FWHandle, hdr, 3);
      while (n = 3) do
        begin
          Op := (hdr[1] * $100) + hdr[0];
          len := hdr[2];
          SetLength (Params, len);
          n := FSFileRead (FWHandle, Params[0], len);
          if (len <> n) then Log ('Data mismatch.');
          AddHCICommand (Op, Params);
          n := FSFileRead (FWHandle, hdr, 3);
        end;
      FSFileClose (FWHandle);
      AddMarker (FIRMWARE_END);
      AddMarker (CLOSE_PORT);
      AddMarker (DELAY_2SEC);
      AddMarker (OPEN_PORT);
    end
  else
    Log ('Error loading Firmware file ' + fn);
end;

procedure BCMEnableRadio (Enable : boolean);
begin
  if Enable then
    AddHCICommand (OGF_VENDOR, $034, [$01])
  else
    AddHCICommand (OGF_VENDOR, $034, [$00]);
end;

function ErrToStr (code : byte) : string;
begin               // page 377 onwards 4.2
  case code of
    $00 : Result := 'Success';
    $01 : Result := 'Unknown HCI Command';
    $02 : Result := 'Unknown Connection Identifier';
    $03 : Result := 'Hardware Failure';
    $04 : Result := 'Page Timeout';
    $05 : Result := 'Authentication Failure';
    $06 : Result := 'PIN or Key Missing';
    $07 : Result := 'Memory Capacity Exceeded';
    $08 : Result := 'Connection Timeout';
    $09 : Result := 'Connection Limit Exceeded';
    $0A : Result := 'Synchronous Connection Limit To A Device Exceeded';
    $0B : Result := 'ACL Connection Already Exists';
    $0C : Result := 'Command Disallowed';
    $0D : Result := 'Connection Rejected due to Limited Resources';
    $0E : Result := 'Connection Rejected due To Security Reasons';
    $0F : Result := 'Connection Rejected due to Unacceptable BD_ADDR';
    $10 : Result := 'Connection Accept Timeout Exceeded';
    $11 : Result := 'Unsupported Feature or Parameter Value';
    $12 : Result := 'Invalid HCI Command Parameters';
    $13 : Result := 'Remote User Terminated Connection';
    $14 : Result := 'Remote Device Terminated Connection due to Low Resources';
    $15 : Result := 'Remote Device Terminated Connection due to Power Off';
    $16 : Result := 'Connection Terminated By Local Host';
    $17 : Result := 'Repeated Attempts';
    $18 : Result := 'Pairing Not Allowed';
    $19 : Result := 'Unknown LMP PDU';
    $1A : Result := 'Unsupported Remote Feature / Unsupported LMP Feature';
    $1B : Result := 'SCO Offset Rejected';
    $1C : Result := 'SCO Interval Rejected';
    $1D : Result := 'SCO Air Mode Rejected';
    $1E : Result := 'Invalid LMP Parameters / Invalid LL Parameters';
    $1F : Result := 'Unspecified Error';
    $20 : Result := 'Unsupported LMP Parameter Value / Unsupported LL Parameter Value';
    $21 : Result := 'Role Change Not Allowed';
    $22 : Result := 'LMP Response Timeout / LL Response Timeout';
    $23 : Result := 'LMP Error Transaction Collision';
    $24 : Result := 'LMP PDU Not Allowed';
    $25 : Result := 'Encryption Mode Not Acceptable';
    $26 : Result := 'Link Key cannot be Changed';
    $27 : Result := 'Requested QoS Not Supported';
    $28 : Result := 'Instant Passed';
    $29 : Result := 'Pairing With Unit Key Not Supported';
    $2A : Result := 'Different Transaction Collision';
    $2B : Result := 'Reserved';
    $2C : Result := 'QoS Unacceptable Parameter';
    $2D : Result := 'QoS Rejected';
    $2E : Result := 'Channel Classification Not Supported';
    $2F : Result := 'Insufficient Security';
    $30 : Result := 'Parameter Out Of Mandatory Range';
    $31 : Result := 'Reserved';
    $32 : Result := 'Role Switch Pending';
    $33 : Result := 'Reserved';
    $34 : Result := 'Reserved Slot Violation';
    $35 : Result := 'Role Switch Failed';
    $36 : Result := 'Extended Inquiry Response Too Large';
    $37 : Result := 'Secure Simple Pairing Not Supported By Host';
    $38 : Result := 'Host Busy - Pairing';
    $39 : Result := 'Connection Rejected due to No Suitable Channel Found';
    $3A : Result := 'Controller Busy';
    $3B : Result := 'Unacceptable Connection Parameters';
    $3C : Result := 'Directed Advertising Timeout';
    $3D : Result := 'Connection Terminated due to MIC Failure';
    $3E : Result := 'Connection Failed to be Established';
    $3F : Result := 'MAC Connection Failed';
    $40 : Result := 'Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock';
    end;
end;

initialization

  SetLength (RxBuffer, 0);
  Queue := MailSlotCreate (1024);
  QueueEvent := TEvent.Create (nil, true, false, '');
  QueueHandle := BeginThread (@QueueHandler, nil, QueueHandle, THREAD_STACK_DEFAULT_SIZE);

end.

