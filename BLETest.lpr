program BLETest;

{$mode objfpc}{$H+}

{$define use_tftp}    // if PI not connected to LAN and set for DHCP then remove this

(****************************************************
May 2017 PJ Design Engineering P/L

Note : Only developed for Pi3 onboard chip.

TODO : Just about everything. Too long to list here.

Current state of Bluetooth system

  Broadacsting seems to be stable.
  Scanning tends to bomb out after a while. Need to try different scan settings.

*****************************************************)

uses
  RaspberryPi3,
  GlobalConfig,
  GlobalConst,
  GlobalTypes,
  Platform,
  Threads,
  SysUtils,
  Classes, Console, Keyboard,
{$ifdef use_tftp}
  uTFTP, winsock2,
{$endif}
  uLog, uBLE,
  Ultibo, uHCI
  { Add additional units here };

type
  TBeacon = class
    uuid : string;
    major, minor : Word;
    rssi : byte;
    count : integer;
  end;

var
  Console1, Console2, Console3 : TWindowHandle;
{$ifdef use_tftp}
  IPAddress : string;
{$endif}
  ch : char;
  beacons : TList;
  no_updates : Longword;
  s : string;

procedure Log1 (s : string);
begin
  ConsoleWindowWriteLn (Console1, s);
end;

procedure Log2 (s : string);
begin
  ConsoleWindowWriteLn (Console2, s);
end;

procedure Log3 (s : string);
begin
  ConsoleWindowWriteLn (Console3, s);
end;

procedure Msg2 (Sender : TObject; s : string);
begin
  Log2 ('TFTP - ' + s);
end;

{$ifdef use_tftp}
function WaitForIPComplete : string;
var
  TCP : TWinsock2TCPClient;
begin
  TCP := TWinsock2TCPClient.Create;
  Result := TCP.LocalAddress;
  if (Result = '') or (Result = '0.0.0.0') or (Result = '255.255.255.255') then
    begin
      while (Result = '') or (Result = '0.0.0.0') or (Result = '255.255.255.255') do
        begin
          sleep (1000);
          Result := TCP.LocalAddress;
        end;
    end;
  TCP.Free;
end;
{$endif}

function GetBeacon (byUUID : string; major, minor : Word) : TBeacon;
var
  i : integer;
  aBeacon : TBeacon;
begin
  Result := nil;
  for i := 0 to Beacons.Count - 1 do
    begin
      aBeacon := TBeacon (Beacons[i]);
      if (CompareText (aBeacon.uuid, byUUID) = 0) and
         (aBeacon.major = major) and (aBeacon.minor = minor) then
        begin
          Result := TBeacon (Beacons[i]);
          exit;
        end;
    end;
end;

function AddBeacon (uuid : string; major, minor : Word) : TBeacon;
begin
  Result := TBeacon.Create;
  Result.uuid := uuid;
  Result.major := major;
  Result.minor := minor;
  Beacons.Add (Result);
end;

procedure WaitForSDDrive;
begin
  while not DirectoryExists ('C:\') do sleep (500);
end;

// called when a marker event is processed by the comms queue
procedure DoMarkerEvent (no : integer);
begin
  case no of
    FIRMWARE_START : Log1 ('Firmware Load Started.');
    FIRMWARE_END   : Log1 ('Firmware Load Completed.');
    OPEN_PORT      : Log1 ('Opening UART0.');
    CLOSE_PORT     : Log1 ('Closing UART0.');
    INIT_COMPLETE  :
      begin
        Log1 ('BLE Chip Initialised.');
        Log1 ('  Name    : ' + ChipName);
        Log1 (format ('  Version : %d Revision : %d', [Ver, Rev]));
        Log1 ('  Address : ' + BDAddrToStr (BDAddr));
      end;
    end;
end;

procedure UpdateBeacons;
var
  i : integer;
  u, mj, mn, si : string;
  aBeacon : TBeacon;
begin
  ConsoleWindowSetXY (Console3, 30, 1);
  ConsoleWindowWrite (Console3, IntToStr (no_updates));
  no_updates := no_updates + 1;
  for i := 0 to 9 do
    begin
      u := '                  ';
      mj := '     ';
      mn := '     ';
      si := '     ';
      if i < Beacons.Count then
        begin
          aBeacon := TBeacon (Beacons[i]);
          u := aBeacon.uuid;
          mj := IntToStr (aBeacon.major);
          mn := IntToStr (aBeacon.minor);
          if aBeacon.rssi = 127 then si := 'NU'
          else if aBeacon.Rssi > 128 then si := '-' + IntToStr (256 - aBeacon.Rssi) + 'dBm'
          else if aBeacon.Rssi <= 20 then si := '+' + IntToStr (aBeacon.Rssi) + 'dBm'
          else si := '??';
        end;
      ConsoleWindowSetXY (Console3, 1, i + 3);
      ConsoleWindowWrite (Console3, u);
      ConsolewindowSetX (Console3, 40);
      ConsoleWindowWrite (Console3, mj);
      ConsolewindowSetX (Console3, 48);
      ConsoleWindowWrite (Console3, mn);
      ConsolewindowSetX (Console3, 56);
      ConsoleWindowWrite (Console3, si);
    end;
end;

// called when a beacon message is detected
procedure DoBeaconEvent (uuid : string; major, minor : Word; Rssi : byte);
var
  aBeacon : TBeacon;
begin
  aBeacon := GetBeacon (uuid, major, minor);
  if aBeacon = nil then aBeacon := AddBeacon (uuid, major, minor);
  aBeacon.rssi := Rssi;
  aBeacon.count := 50;    // 5 seconds
  UpdateBeacons;
end;

begin
  Console1 := ConsoleWindowCreate (ConsoleDeviceGetDefault, CONSOLE_POSITION_LEFT, true);
  Console2 := ConsoleWindowCreate (ConsoleDeviceGetDefault, CONSOLE_POSITION_TOPRIGHT, false);
  Console3 := ConsoleWindowCreate (ConsoleDeviceGetDefault, CONSOLE_POSITION_BOTTOMRIGHT, false);
  SetLogProc (@Log1);
  no_updates := 0;
  Beacons := TList.Create;
  Log1 ('Bluetooth (BLE) TEST.');
  SetLog3Proc (@log3);
  Log3 ('Beacons');
  WaitForSDDrive;

{$ifdef use_tftp}
  IPAddress := WaitForIPComplete;
  Log2 ('TFTP - Enabled.');
  Log2 ('TFTP - Syntax "tftp -i ' + IPAddress + ' PUT kernel7.img"');
  SetOnMsg (@Msg2);
  Log2 ('');
{$endif}

  SetMarkerEvent (@DoMarkerEvent);       // set marker event (called when marker processed on event queue)
  SetBeaconEvent (@DoBeaconEvent);       // set beacon event (called when beacon is detected)
  AddMarker (OPEN_PORT);                 // open uart
  AddMarker (DELAY_50MSEC);              // ensure read thread has started
  ResetChip;                             // reset chip
  BCMLoadFirmware ('BCM43430A1.hcd');    // load firmware
  ReadLocalName;                         // read new chip name
  ReadLocalVersion;                      // read new HCI version
  ReadBDADDR;                            // read newly assigned BD address
  AddMarker (INIT_COMPLETE);             // indicate initialisation complete

  Log2 ('Commands...');
  Log2 ('  P  Start Passive Scanning. (Still in development)');
  Log2 ('  A  Start Active Scanning. (Still in development)');
  Log2 ('  S  Stop Scanning.');
  Log2 ('  I  Start Inquiry. (Still in development)');
  Log2 ('');
  Log2 ('  U  Start Undirected Broadcasting.');
  Log2 ('  B  Start Broadcasting as iBeacon.');
  Log2 ('  X  Stop Broadcasting.');
  Log2 ('');
  Log2 ('  C  Clear Window (about the only thing not in development).');
  Log2 ('');

  while true do
    begin
      if ConsoleGetKey (ch, nil) then
        case uppercase (ch) of
          'I' : Inquiry;
          'P' : StartPassiveScanning;
          'A' : StartActiveScanning;
          'S' : StopScanning;
          'C' : ConsoleWindowClear (Console1);
          'U' :
            begin
              s := 'Flange Modulator';
              Log1 ('Broadcasting as "' + s + '".');
              ClearAdvertisingData;
              AddAdvertisingData (ADT_FLAGS, [$1a]);
              AddAdvertisingData (ADT_COMPLETE_LOCAL_NAME, s);
              StartUndirectedAdvertising;
            end;
          'X' : StopAdvertising;
          'B' :
            begin
              Log1 ('Broadcasting as iBeacon.');
              ClearAdvertisingData;
              AddAdvertisingData (ADT_FLAGS, [$1A]);
              AddAdvertisingData (ADT_MANUFACTURER_SPECIFIC,
                    [lo (ID_APPLE), hi (ID_APPLE), // company identifier
                     $02, $15,                     // advertisement indicator
                     $63, $6F, $3F, $8F, $64, $91, $4B, $EE, $95, $F7, $D8, $CC, $64, $A8, $63, $B5, // our iBeacon proximity uuid
                     $00, $01,                     // Major
                     $00, $02, 	                   // Minor
                     $C8, $00]); 	                 // Calibrated Tx power
              StartUndirectedAdvertising;
            end;
        end;
    end;
  ThreadHalt (0);
end.

