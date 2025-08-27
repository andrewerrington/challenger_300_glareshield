/*
   Challenger 300 Glare Shield devices

   Copyright (c) November 2023 A M Errington

   v0.2 26 Apr 2025 - Modified for new schematic, which allows A0 to be used.
   v0.3 16 Jul 2025 - Corrected operation of pitch wheel in VS and PTCH mode.

   This code is to interface switches and indicators on the Challenger 300
   glare shield panel to X-Plane. We use a Wiznet W5500 Ethernet module
   to communicate with X-Plane over UDP.

   In the glare shield, from the left (Pilot's side) is a Master Warning/
   Master Caution indicator and switch.
   Next is the Pilot's Display Control Panel (DCP) which has ten pushbuttons
   and three dual concentric rotary encoders with pushbuttons.
   Next, in the centre is the Flight Guidance Panel (FGP) which has 14
   pushbuttons, five rotary encoders with pushbuttons, a scroll wheel rotary
   encoder, a slide switch.
   Next is the First Officer's DCP, identical to the Pilot's.
   Finally, is the First Officer's Master Caution indicator and switch,
   identical to the Pilot's.

   The two DCPs and FGP are backlit.

   The total width of the glare shield is approx. 44-3/4" (1140mm).

   All five panels will have input devices wired in a matrix, with the
   Arduino near the centre. At most, only two switches or rotary encoders
   will be active at any one time (one by Pilot, one by FO). However, some
   inputs may be held in an active state over time (e.g. some rotary
   encoders, and the AP/YD DISC switch), therefore all matrix nodes are
   protected with diodes.
   The matrix must be scanned rapidly to capture rotary encoder changes.
   The matrix will be constructed with six Arduino I/Os for the rows,
   and an MCP23017 I2C GPIO expander for the columns.
   The MCP23017 is a 16-bit device, but only 7 bits on each port can be used
   as inputs. Therefore we create a 6x14 matrix, 84 switches.

   There are four outputs needed:
   Master Warning, PWM driving a red LED at either end of the glare shield.
   Master Caution, PWM driving a yellow LED at either end of the glare shield.
   Zone 1 Lighting. PWM driving lighting in the following areas:
     Pilot glare shield illumination (left)
     Pilot DCP backlight
     Centre FGP backlight
     Compass illumination
   Zone 3 Lighting. PWM driving lighting in the following areas:
     FO glare shield illumination (right)
     FO DCP backlight
*/

//        1         2         3         4         5         6         7
// 34567890123456789012345678901234567890123456789012345678901234567890123456789

// Support for Wiznet W5500 Ethernet module
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// Ethernet MAC address, must be unique for all devices on network.
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// This code uses DHCP. Your network should assign an IP address to this unit.
// Alternatively, check the Arduino Ethernet examples to show how to set a
// fixed IP address in this code.

// This code also listens for the X-Plane multicast beacon to get the
// IP address of the X-Plane PC. If you can't use the beacon, then set the
// IP address of your X-Plane PC on the network here, and remove the beacon
// code.

IPAddress xplane_ip(0,0,0,0); // Overwritten by the beacon address later.

IPAddress BECN_ip(239,255,1,1); // UDP multicast address for X-Plane beacon.
unsigned int BECN_port = 49707; // X-Plane beacons on this port (was 49000).

unsigned int xplane_port = 49000; // X-Plane listens on this port.

unsigned int localPort = 49094; // local port we listen on (can be any port).

// An EthernetUDP instance to let us send and receive packets over UDP.
EthernetUDP udp;

// Buffer for receiving and sending UDP data.
char packetBuffer[512];


// Support for MCP20317 I2C GPIO Expander
#include <Wire.h>
#include <MCP23017.h>

// Default MCP23017 address
#define MCP23017_ADDR 0x20
MCP23017 mcp = MCP23017(MCP23017_ADDR);


// LED outputs. Must be PWM capable for Challenger 300.
const int zone1_lighting_pin = 3;
const int zone3_lighting_pin = 5;
const int warning_LED_pin = 6;
const int caution_LED_pin = 9;

// Variable to hold new brightness value from sim
int16_t brightness;


// Input matrix
// Rows are on Arduino pins. Columns on MCP23017 port expander.
uint8_t rows[] = {4, 7, 8, A1, A2, A3};

// An array of ints to make a bit store.
// We fetch 16-bits from the port expander, even though we only use 14.
uint16_t cur_matrix[sizeof rows / sizeof rows[0]];
// We also have an array of old bit-states to detect changes.
uint16_t prev_matrix[sizeof rows / sizeof rows[0]];


// For testing we can hook up a button to D2.
const int test_button_pin = 2;
bool test_button_pressed = false;
bool old_test_button_pressed = false;
bool debug = false;

// Long interval timer 1s
unsigned long slow_tick_start_millis;
unsigned long slow_tick_current_millis;
const unsigned long slow_tick_period = 1000;

// Short interval timer (currently no tasks)
unsigned long fast_tick_start_millis;
unsigned long fast_tick_current_millis;
const unsigned long fast_tick_period = 100;


// 1ms interrupt tells us to sample the matrix via this flag
volatile bool scan_matrix;


// A union to convert between bytes and floats
union u_tag {
  byte bin[4];
  float num;
} u;


// -----------------------------------------------------------------------------

bool getCurBit(uint8_t n) {
  // Get bit n from the current bit array.
  // Treat the array as 6x16 even though it's really only 6x14 because
  // we aren't using the MSB of the two GPIO ports.
  // Bits are stored in row order 0 to 5, in 16-bit chunks (b << 8 | a)
  // To calculate the bit number for a row/column pair:
  // (R * 16) + C for group A
  // (R * 16) + C + 8 for group B
  return (cur_matrix[n / 16] & 1 << (n % 16));
}

bool getPrevBit(uint8_t n) {
  // Get bit n from the previous bit array (like getCurBit()).
  return (prev_matrix[n / 16] & 1 << (n % 16));
}

void putPrevBit(uint8_t n, bool v) {
  // Store bit value v into bit n of the previous bit array.
  if (v){
    // New bit is set, OR it into place
    prev_matrix[n / 16] |= (1 << (n % 16));
  }
  else
  {
    // New bit is clear, mask it out
    prev_matrix[n / 16] &= ~(1 << (n % 16));
  }
}


// Classes for various input devices. Rotary encoder, pushbutton, SPST/SPDT
// switch, SP3T (with centre position), rotary switch (up to 12 positions).
// Todo: Add a class for potentiometer input.
// fixme: remove switch class and potentiometer class. Move them to demo code.

// The matrix is built with Arduino GPIO pins as rows, and MCP23017 GPIO
// pins as columns. At each intersection is a diode-isolated switch.

// A rotary encoder is just two switches, which change state as the encoder
// is turned.
// A pushbutton is a single switch that disconnects when it is released, aka
// a momentary switch.
// A switch is a switch that stay in place until moved to a new position, i.e.
// a latching switch.
// A rotary switch has two or more discrete positions, but only one is active
// at any time.

// The switches are wired into the matrix, which is sampled every millisecond.
// After sampling, each switch should be examined to see if it has changed
// state. If so, perform some action such as sending a command, or updating
// a dataref.

class Encoder4
{
    // Rotary Encoder input, four states per click.
    // Common pin, C, is connected to a column. Encoder pins, A & B, are
    // connected to two *different* rows.
    // At startup we assume nothing about the encoder's state.
    // Handle the encoder pushbutton (if there is one) as an independent
    // Button object elsewhere.

    // Class Member Variables
    uint8_t A;        // bit in matrix for encoder A pin
    uint8_t B;        // bit in matrix for encoder B pin

    int16_t intlValue; // internal encoder count
    //int8_t intlDelta;  // internal encoder change

  public:
    int8_t delta;  // External last change in value, -1, 0, +1
    //int16_t value;  // External encoder count. Always intlValue/4
    int16_t lastValue; // Last external value

    // Constructor - creates a rotary encoder and initializes the member
    // variables and state

    Encoder4(uint8_t bit_A, uint8_t bit_B) {
      // fixme: Could these be constants, set at compile time?
      A = bit_A;
      B = bit_B;

      intlValue = 0;
      //value = 0;
      lastValue = 0;
      delta = 0;
      //intlDelta = 0;
    }

    void tick() {
      int8_t intlDelta;  // internal encoder change
      // Update the encoder state, based on the bits in the input matrix
      uint8_t newState = ((getPrevBit(B) << 3) | (getPrevBit(A) << 2) | ((getCurBit(B) << 1) | getCurBit(A)) & 0x0F);
      // newState now contains a 4-bit value that can be used to update the
      // encoder value. We expect the 2-bit inputs to change thus:
      // 00 -> 01 -> 11 -> 10 -> 00  + direction
      // 00 -> 10 -> 11 -> 01 -> 00  - direction

      //      OldNew
      //       BABA
      // 0x00  0000  No change
      // 0x01  0001  +1
      // 0x02  0010  -1
      // 0x03  0011  Illegal
      // 0x04  0100  -1
      // 0x05  0101  No change
      // 0x06  0110  Illegal
      // 0x07  0111  +1

      // 0x08  1000  +1
      // 0x09  1001  Illegal
      // 0x0A  1010  No change
      // 0x0B  1011  -1
      // 0x0C  1100  Illegal
      // 0x0D  1101  -1
      // 0x0E  1110  +1
      // 0x0F  1111  No change

      intlDelta = 0;

      if ((newState == 0x01) || (newState == 0x07)
          || (newState == 0x08) || (newState == 0x0E)) intlDelta = 1;
      if ((newState == 0x02) || (newState == 0x04)
          || (newState == 0x0B) || (newState == 0x0D)) intlDelta = -1;

      intlValue += intlDelta;  // Update the internal value

      //value = intlValue / 4; // Four states per click

      delta = (intlValue/4) - lastValue;

      lastValue = intlValue/4;

      putPrevBit(B, newState & 0x02);
      putPrevBit(A, newState & 0x01);

    }

};  // class Encoder4


class Encoder2
{
    // Rotary Encoder input, two states per click.
    // Common pin, C, is connected to a column. Encoder pins, A & B, are
    // connected to two *different* rows.
    // At startup we assume nothing about the encoder's state.
    // Handle the encoder pushbutton (if there is one) as an independent
    // Button object elsewhere.

    // Class Member Variables
    uint8_t A;        // bit in matrix for encoder A pin
    uint8_t B;        // bit in matrix for encoder B pin

    int16_t intlValue; // internal encoder count
    int8_t intlDelta;  // internal encoder change

  public:
    int8_t delta;  // External last change in value, -1, 0, +1
    int16_t value;  // External encoder count
    int16_t lastValue; // Last ezoxternal value

    // Constructor - creates a rotary encoder and initializes the member
    // variables and state

    Encoder2(uint8_t bit_A, uint8_t bit_B) {
      // fixme: Could these be constants, set at compile time?
      A = bit_A;
      B = bit_B;

      intlValue = 0;
      value = 0;
      lastValue = 0;
      delta = 0;
      intlDelta = 0;
    }

    void tick() {
      // Update the encoder state, based on the bits in the input matrix

      uint8_t newState = ((getPrevBit(B) << 3) | (getPrevBit(A) << 2) | ((getCurBit(B) << 1) | getCurBit(A)) & 0x0F);
      // newState now contains a 4-bit value that can be used to update the
      // encoder value. We expect the 2-bit inputs to change thus:
      // 00 -> 01 -> 11 -> 10 -> 00  + direction
      // 00 -> 10 -> 11 -> 01 -> 00  - direction

      //      OldNew
      //       BABA
      // 0x00  0000  No change
      // 0x01  0001  +1
      // 0x02  0010  -1
      // 0x03  0011  Illegal
      // 0x04  0100  -1
      // 0x05  0101  No change
      // 0x06  0110  Illegal
      // 0x07  0111  +1

      // 0x08  1000  +1
      // 0x09  1001  Illegal
      // 0x0A  1010  No change
      // 0x0B  1011  -1
      // 0x0C  1100  Illegal
      // 0x0D  1101  -1
      // 0x0E  1110  +1
      // 0x0F  1111  No change

      intlDelta = 0;

      if ((newState == 0x01) || (newState == 0x07)
          || (newState == 0x08) || (newState == 0x0E)) intlDelta = 1;
      if ((newState == 0x02) || (newState == 0x04)
          || (newState == 0x0B) || (newState == 0x0D)) intlDelta = -1;

      intlValue += intlDelta;  // Update the internal value

      value = intlValue / 2; // Four states per click

      delta = value - lastValue;

      lastValue = value;

      putPrevBit(B, newState & 0x02);
      putPrevBit(A, newState & 0x01);

    }

};  // class Encoder2


class Button
{
    // Button input.
    // Connected between a row and column.
    // At startup we assume the button is not pressed.

    // Class Member Variables
  private:
    const uint8_t n;      // bit in matrix for switch. fixme: Can this be a constant?
    //unsigned long previousMillis;   // use for debouncing later if necessary

    // Constructor - creates a button
  public:
    int8_t delta; // Last change, +1 button became pressed,
    // -1 button became released, 0 no change.

    Button(uint8_t bit_n) : n(bit_n) {
      //n = bit_n;
      delta = 0;
    }

    void tick() {
      bool currState;
      bool prevState;
      // Check for button state change
      currState = getCurBit(n);
      prevState = getPrevBit(n);

      if (currState == prevState)
        delta = 0;
      else
        delta = (currState ? 1 : -1);

      putPrevBit(n, currState);
    }

};  // class Button


class SwitchMulti
{
    // Switch input. fixme: implement this.
    // Used for SPDT, SP3T, and rotary switches.
    // Connected between a single row and multiple columns.
    // The switch can have several states, based on which column is connected,
    // including the state where no column is connected. This is particularly
    // important for 3-position toggle switches, which usually have a 'centre
    // off' position. We can't detect this electrically, but we can assume that
    // if no other position is connected then we are in the centre position.
    // A word of caution however, if a switch is quickly moved from one position
    // to another it may disconnect for long enough to be detected as a 'centre
    // off' position on its way to the next switch position.

    // For simple on/off toggle switches

    // Important: At startup we assume the switch has changed, which will cause
    // a command or dataref to be sent to X-Plane so that it 'synchronises'
    // with the real switch.

    // Class Member Variables
    uint8_t n;      // bit in matrix for switch (todo: this needs to be many. Suggest 4-bit field)

    unsigned long previousMillis;   // use for debouncing later if necessary

    // Constructor - creates a button
  public:
    int16_t currState;  // Currently selected position
    uint8_t prevState;  // Previously selected position
    int8_t delta; // Last change, +1 switch changed, 0 no change.

    // Need a variable number of arguments here...
    SwitchMulti(uint8_t bit_n) {
      n = bit_n;
      currState = 0;
      prevState = 0;
      delta = 0;
    }

    void tick() {
      // Check for button state change
      currState = getCurBit(n);

      if (currState == prevState)
        delta = 0;
      else
        delta = (currState ? 1 : -1);

      prevState = currState;
    }

};  // class switchMulti



// Declare encoders.
// DCP has 3 dual concentric rotary encoders (total 6 encoders).
// There are two DCPs, Pilot (left) on A matrix, FO (right) on B matrix.

// DCP Pilot panel
Encoder2 dcp_p_tune_outer(37,69); // Command xap/DCP/dcp_tune_left_coarse _right_coarse
Encoder2 dcp_p_tune_inner(5,21); // Command xap/DCP/dcp_tune_left_fine _right_fine
Encoder2 dcp_p_menu(36,68);   // Command xap/DCP/dcp_menu_left _right
Encoder2 dcp_p_data(4,20);   // Command xap/DCP/dcp_data_left _right
Encoder2 dcp_p_tilt(35,67);   // Change cl300/dcp_tilt CCW -1 CW +1 -1500 to 1500
int tilt_value = 0;// fixme: initialise this by querying sim
Encoder2 dcp_p_range(3,19);  // Change cl300/dcp_range CCW -1 CW +1 -150 to 150
int range_value = 0;// fixme: initialise this by querying sim

// DCP FO panel. Commands are exactly the same as DCP. Sim handles both sides
// the same.
Encoder2 dcp_fo_tune_outer(45,77);
Encoder2 dcp_fo_tune_inner(13,29);
Encoder2 dcp_fo_menu(44,76);
Encoder2 dcp_fo_data(12,28);
Encoder2 dcp_fo_tilt(43,75);
Encoder2 dcp_fo_range(11,27);

// FGP has 5 regular encoders, and a scroll wheel.
// Sim handles Pilot CRS knob and FO CRS knob the same.
Encoder4 fgp_p_crs(18, 2);  // R0A2 R1A2 Command sim/radios/obs_HSI_down _up
Encoder4 fgp_fo_crs(26, 10);  // R0B2 R1B2

Encoder4 fgp_hdg(17, 1);    // R0A1 R1A1 Command sim/autopilot/heading_down _up
Encoder4 fgp_speed(16, 0);  // R0A0 R1A0 Command sim/autopilot/airspeed_down _up
Encoder4 fgp_alt(25, 9);    // R0B1 R1B1 Command cl300/autop/autop_alt_dial_dn _up

// Pitch wheel is two states per click.
Encoder2 fgp_pitch(8, 24);  // R0B0 R1B0 Command sim/autopilot/vertical_speed_up _dn


// Declare buttons
// MCP has one button. Sim handles both sides as one input.
// Pilot side
Button p_warning_caution_clear(85);  // R5A5 Command sim/annuniciator/clear_master_warning *_caution
// FO side
Button fo_warning_caution_clear(93); // R5B5


// FGP has 19 buttons, including 5 rotary pushbuttons
// Sim handles Pilot FD button and FO FD button the same.
Button fgp_p_fd(81);      // R5A1 Command sim/autopilot/fdir_toggle
Button fgp_p_push_direct(49); // R3A1 Set cl300/crc_butt to 1, then zero
Button fgp_fo_fd(89);     // R5B1
Button fgp_fo_push_direct(57); // R3B1

Button fgp_nav(80);       // R5A0 Command sim/autopilot/NAV
Button fgp_btn_hdg(66);   // R4A2 Command sim/autopilot/heading
Button fgp_appr(65);      // R4A1 Command sim/autopilot/approach

Button fgp_half_bank(48); // R3A0 Toggle cl300/half_bank_h 0/1
bool half_bank_value = false;  // fixme: initialise this by querying sim

Button fgp_push_sync(34); // R2A2 Command Set cl300/fgp_s_hdg to 1, then zero
Button fgp_b_c(33);       // R2A1 Command sim/autopilot/back_course

Button fgp_flc(64);       // R4A0 Command sim/autopilot/level_change & speed_hold
Button fgp_push_ias_mach(32);  // R2A0 Command sim/autopilot/knots_mach_toggle

Button fgp_vs(72);        // R4B0 Command sim/autopilot/vertical_speed
Button fgp_vnav(40);      // R2B0 Command sim/autopilot/FMS
char vvi_status = 0;      // VS button changes mode in sim, which changes pitch wheel commands
                          // Value can be 2 (VS mode) or 0 (PTCH mode)


Button fgp_btn_alt(73);   // R4B1 Command sim/autopilot/altitude_hold
Button fgp_push_cancel(41); // R2B1 ? Not present in sim

Button fgp_ap(74);        // R4B2 Command sim/autopilot/servos_toggle & fdir_servos_toggle & autothrottle_off?
Button fgp_yd(88);        // R5B0 Command sim/systems/yaw_damper_toggle
Button fgp_xfr(42);       // R2B2 Toggle cl300/autop_xfr_h
bool xfr_value = false;   // fixme: initialise this by querying sim


// Declare switches
// FGP has one switch
// Switch class not implemented yet. However, this switch has only one state, so
// we can handle it as a button with only a 'pressed' action.
Button fgp_ap_yd_disc(56);  // R3B0 Command sim/autopilot/servos_fdir_yawd_off


// DCP has 13 buttons, including 3 rotary pushbuttons
// There are two identical DCPs, one on A matrix, one on B matrix

// Pilot side

Button dcp_p_1_2(70);    // Toggle cl300/dcp_1_2
bool dcp_1_2_value = false;   // fixme: initialise this by querying sim

Button dcp_p_swap(22);   // Command xap/DCP/dcp_tune_stb
Button dcp_p_dme_h(6);  // Toggle cl300/dcp/dmeh
bool dme_h_value = false; // fixme: initialise this by querying sim

Button dcp_p_radio(38);  // Toggle cl300/mfd_sel_state_h
bool radio_value = false; // fixme: initialise this by querying sim

Button dcp_p_nav_src(86);  // Command cl300/DCP/navsrc_toggle
Button dcp_p_brg_src(54);  // Command cl300/DCP/brgsrc_toggle
Button dcp_p_select(53); // Command xap/DCP/dcp_data_sel
Button dcp_p_frmt(84);   // Command sim/instruments/EFIS_mode_up
Button dcp_p_refs(52);   // cl300/DCP/dcp_refs_button
Button dcp_p_tfc(83);    // Toggle sim/cockpit/switches/EFIS_shows_tcas
bool tfc_value = false; // fixme: initialise this by querying sim

Button dcp_p_radar(51);  // Toggle cl300/dcp_radar
bool radar_value = false; // fixme: initialise this by querying sim

Button dcp_p_tr_wx(82);  // Command cl300/tr_wx
Button dcp_p_push_auto(50);  // fixme: action unknown

// FO side
Button dcp_fo_1_2(78);
Button dcp_fo_swap(30);
Button dcp_fo_dme_h(14);
Button dcp_fo_radio(46);
Button dcp_fo_nav_src(94);
Button dcp_fo_brg_src(62);
Button dcp_fo_select(61);
Button dcp_fo_frmt(92);
Button dcp_fo_refs(60);
Button dcp_fo_tfc(91);
Button dcp_fo_radar(59);
Button dcp_fo_tr_wx(90);
Button dcp_fo_push_auto(58);

// End encoders and buttons declaration

//void printBin(uint16_t x) {
//  for (int8_t i = 15; i >= 0; i--)
//    Serial.write(bitRead(x, i) ? '1' : '0');
//}


int sendUDP(uint16_t bytes) {
  udp.beginPacket(xplane_ip, xplane_port);
  udp.write(packetBuffer, bytes);
  return udp.endPacket();
}


void subscribeDataref(const char* dataref, uint16_t freq_Hz, uint8_t index) {
  // Create and send UDP packet to subscribe to a dataref.
  // First argument is a PSTR()
  // e.g.
  // subscribeDataref(PSTR("sim/cockpit2/switches/panel_brightness_ratio[0]"),4,4);

  // General subscription packet is of the form
  // "RREF\x00aaaabbbbpath\to\dataref"
  // Where aaaa is a 32-bit integer representing the desired reporting frequency
  // and bbbb is a 32-bit integer used as an index to tag incoming data so that
  // we know which dataref it refers to. Here they are limited to 16-bits.
  Serial.println(F("Clear packet buffer."));
  memset(packetBuffer, 0, sizeof(packetBuffer)); // Clear the packet buffer
  // Build the request in the packet buffer.
  Serial.println(F("Build packet buffer."));
  strcpy_P(packetBuffer, PSTR("RREF"));
  memcpy(packetBuffer + 5, &freq_Hz, sizeof(freq_Hz));
  memcpy(packetBuffer + 9, &index, sizeof(index));
  Serial.println(F("Copy dataref."));
  strcpy_P(packetBuffer + 13, dataref);
  Serial.println(F("Send."));
  if (!debug) {
    sendUDP(413); // Subscription packet is 413 bytes.
  }
  Serial.println(F("Done."));
}


void writeDataref(const char* dataref, float val) {
  // Create and send UDP packet to write a float value to a dataref.
  // First argument is a PSTR()
  // e.g.
  // writeDataref(PSTR("sim/cockpit2/switches/panel_brightness_ratio[0]"),0.5);

  // General write dataref packet is of the form
  // "DREF\x00nnnnpath\to\dataref"
  // Where nnnn is a 32-bit float to be written to the dataref.
  Serial.println(F("Clear packet buffer."));
  memset(packetBuffer, 0, sizeof(packetBuffer)); // Clear the packet buffer
  // Build the request in the packet buffer.
  Serial.println(F("Build packet buffer."));
  strcpy_P(packetBuffer, PSTR("DREF"));
  memcpy(packetBuffer + 5, &val, sizeof(val));
  Serial.println(F("Copy dataref."));
  strcpy_P(packetBuffer + 9, dataref);
  Serial.println(F("Send."));
  if (!debug) {
    sendUDP(509); // Dataref packet is 509 bytes.
  }
  Serial.println(F("Done."));
}


void sendCommand(const char* command) {
  // Create and send UDP packet to for an X-Plane command.
  // Argument is a PSTR()
  // e.g.
  // sendCommand(PSTR("sim/annunciator/clear_master_caution"));

  // General command packet is of the form
  // "CMND\x00path\to\command\x00"

  // No need to clear the packet buffer as we will send exactly what we put in.
  // Build the command in the packet buffer. PSTR contains the trailing \x00.
  Serial.println(F("Build packet buffer."));
  strcpy_P(packetBuffer, PSTR("CMND"));
  Serial.println(F("Copy command."));
  strcpy_P(packetBuffer + 5, command);
  Serial.println(F("Send."));
  if (!debug) {
    sendUDP(strlen_P(command) + 6); // Command packet is only as long as needed.
  }
  Serial.println(F("Done."));
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();
  Serial.println(F("Challenger 300 Glare Shield Interface v0.3"));

  // Test switch input pin
  pinMode(test_button_pin, INPUT_PULLUP);
  debug=!digitalRead(test_button_pin);
  // Use our precious test button to turn on debug mode. Currently this
  // is used to prevent network communication as UDP seems to get stuck
  // for a few seconds if the host is not present.
  if (debug) {
    Serial.println(F("Debug mode on."));
  }

  // Set up Ethernet
  // Times out after 1 minute if no DHCP server responds.
if (!debug) {
  Serial.println(F("Setting up Ethernet. Waiting for DHCP."));
  Ethernet.begin(mac);

  Serial.print(F("IP address: "));
  Serial.println(Ethernet.localIP());

  Serial.println(F("Listening for multicast beacon."));
  udp.beginMulticast(BECN_ip, BECN_port);

  while (true){
    int packetSize = udp.parsePacket();
    if (packetSize){
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      udp.read(packetBuffer, packetSize);

      if (strncmp(packetBuffer, "BECN\x00", 5) == 0) {
        Serial.println(F("Packet is BECN."));
        Serial.println(F("X-Plane IP"));
        Serial.print(udp.remoteIP());
        xplane_ip = udp.remoteIP();
        // We have the address, break out. Otherwise loop forever, because we
        // don't know the address of X-Plane
        break;
      }
    }
    delay(10);
  }

  // start UDP
  udp.begin(localPort);

  } // debug
  
  // Set up MCP23017
  // MCP23017 is used for column inputs in the switch matrix
  mcp.init();

  // Set both ports to inputs with pull-ups
  // We're not supposed to use the MSB as input
  mcp.portMode(MCP23017Port::A, 0x7F, 0x7F);
  mcp.portMode(MCP23017Port::B, 0x7F, 0x7F);

  mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A
  mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

  // Set up rows as inputs with pull-up
  for (uint8_t i = 0; i < sizeof rows / sizeof rows[0]; i++) {
    pinMode(rows[i], INPUT_PULLUP);
    digitalWrite(rows[i], HIGH);
  }

  // Set up Timer0 for 1ms interrupt
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  // 1ms interrupt will set this flag, so we know it's time to scan the matrix.
  scan_matrix = false;

  // Power-on test if button pressed
  if (!digitalRead(test_button_pin)){

    Serial.println(F("Self test."));
  
    // Fade up Zone 1 lighting then Zone 3 lighting
    Serial.println(F("Fade up Zone 1"));
    for (uint8_t i = 0; i < 255; i++) {
      analogWrite(zone1_lighting_pin, i);
      delay(5);
    }
  
    Serial.println(F("Fade up Zone 3"));
    for (uint8_t i = 0; i < 255; i++) {
      analogWrite(zone3_lighting_pin, i);
      delay(5);
    }
  
    // Fade up WARNING then CAUTION LEDs
    Serial.println(F("Fade up WARNING"));
    for (uint8_t i = 0; i < 255; i++) {
      analogWrite(warning_LED_pin, i);
      delay(5);
    }
  
    Serial.println(F("Fade up CAUTION"));
    for (uint8_t i = 0; i < 255; i++) {
      analogWrite(caution_LED_pin, i);
      delay(5);
    }
  
    // Fade everything down
    Serial.println(F("Fade down"));
    for (uint8_t i = 255; i > 0; i--) {
      analogWrite(zone1_lighting_pin, i - 1);
      analogWrite(zone3_lighting_pin, i - 1);
      analogWrite(warning_LED_pin, i - 1);
      analogWrite(caution_LED_pin, i - 1);
      delay(5);
    }
  
  
    // Reset lighting to 5%
    analogWrite(zone1_lighting_pin, 13);
    analogWrite(zone3_lighting_pin, 13);
    delay(500);
  
    Serial.println(F("Self test done."));
  }

  // Unsubscribe in case we have been reset.
  Serial.println(F("Unsubscribing from datarefs."));
  subscribeDataref(PSTR("cl300/mast_warn"), 0, 0x01);
  subscribeDataref(PSTR("cl300/mast_caut"), 0 , 0x02);
  subscribeDataref(PSTR("cl300/gshldl"), 0, 0x03);
  subscribeDataref(PSTR("cl300/gshldr"), 0, 0x04);

  Serial.println(F("Subscribing to datarefs."));

  // Subscribe to Master Warning, 20 per second, with an index of 0x01
  subscribeDataref(PSTR("cl300/mast_warn"), 20, 0x01);

  // Subscribe to Master Caution, 20 per second, with an index of 0x02
  subscribeDataref(PSTR("cl300/mast_caut"), 20 , 0x02);

  // Subscribe to gshldl, five times per second, with an index of 0x03.
  // This will be Zone1
  subscribeDataref(PSTR("cl300/gshldl"), 5, 0x03);

  // Subscribe to gshldr, five times per second, with an index of 0x04.
  // This will be Zone3
  subscribeDataref(PSTR("cl300/gshldr"), 5, 0x04);

  // Subscribe to vvi_status, two times per second, with an index of 0x05.
  // This will be used internally to change what the Pitch Wheel does.
  subscribeDataref(PSTR("sim/cockpit2/autopilot/vvi_status"), 2, 0x05);


  fast_tick_start_millis = millis();
  slow_tick_start_millis = millis();

  Serial.println(F("setup() done."));

} // setup()


// 1ms interrupt handler.
SIGNAL(TIMER0_COMPA_vect) {
  // Set a flag to cause the main loop to scan the matrix
  scan_matrix = true;
}


void loop() {

  int packetSize = 0;

  // Test button can be used for various things. True if pressed.
  test_button_pressed = (!digitalRead(test_button_pin));
  if (test_button_pressed != old_test_button_pressed) {
    Serial.println((test_button_pressed)?F("Test button pressed"):F("Test button released"));
    old_test_button_pressed = test_button_pressed;
  }

  // Update the 1-second timer
  slow_tick_current_millis = millis();
  if (slow_tick_current_millis - slow_tick_start_millis >= slow_tick_period) {
    // We are on a 1-second boundary
    // Do something useful
    Serial.println(F("B A0 B A1 B A2 B A3 B A4 B A5"));
    for (uint8_t i = 0; i < (sizeof rows / sizeof rows[0]); i++) {
      Serial.print(cur_matrix[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
    for (uint8_t i = 0; i < (sizeof rows / sizeof rows[0]); i++) {
      Serial.print(prev_matrix[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();

    // A0 is now available
    Serial.print(F("A0: "));
    Serial.println(analogRead(A0));

    // Reset the timer
    slow_tick_start_millis = slow_tick_current_millis;
  }

  // Update the 0.1-second timer
  fast_tick_current_millis = millis();
  if (fast_tick_current_millis - fast_tick_start_millis >= fast_tick_period) {
    // We are on a 0.1-second boundary
    // Nothing to do in this version.

    // Reset the timer
    fast_tick_start_millis = fast_tick_current_millis;
  }

  // Check for incoming UDP packet
  if (!debug) {
    packetSize = udp.parsePacket();
  }
  if (packetSize) {
    if (debug) {  // fixme: need another debug flag for this
    Serial.print(F("Received "));
    Serial.print(packetSize);
    Serial.print(F(" byte packet from "));
    Serial.print(udp.remoteIP());
    Serial.print(F(":"));
    Serial.println(udp.remotePort());
    }

    // read the packet into packetBuffer
    udp.read(packetBuffer, packetSize);
    if (debug) {
    Serial.println(F("Contents:"));
    for (int i = 0; i < packetSize; i++) {
      Serial.print((uint8_t)packetBuffer[i], HEX);
      Serial.print(" ");
    
      Serial.println();

      for (int i = 0; i < packetSize; i++) {
        if (packetBuffer[i] < 32) {
          Serial.print("\\x");
          if (packetBuffer[i] < 16) Serial.print("0");
          Serial.print((uint8_t)packetBuffer[i], HEX);
        }
        else
          Serial.print(packetBuffer[i]);
      }
      Serial.println();
    }
    }
    
    if (strncmp(packetBuffer, "RREF", 4) == 0) {
      //Serial.println(F("Packet is RREF"));
      // Currently we are only expecting "RREF," packets.
      // Start at position 5 and unpack the rest
      for (int i = 5; i < packetSize; i += 8) {
        // Unpack the dataref value. It's always a float, so assemble these
        // four bytes into a float
        u.bin[0] = packetBuffer[i + 4];
        u.bin[1] = packetBuffer[i + 5];
        u.bin[2] = packetBuffer[i + 6];
        u.bin[3] = packetBuffer[i + 7];
        float out = u.num;
        // Check the index value, which will be whatever we used when
        // subscribing. It's a 32-bit value, but we only need the first
        // byte as we will only subscribe to a small number of items.

        // Index 1 is Warning
        // Index 2 is Caution
        // Index 3 is Zone 1 brightness
        // Index 4 is Zone 3 brightness

        switch (packetBuffer[i]) {
          case 1: // WARNING indicator
            //Serial.print(F("Warning: "));
            //Serial.println(out);
            brightness = out * 255.0;
            brightness = constrain(brightness, 0, 255);
            analogWrite(warning_LED_pin, brightness);
            break;
          case 2: // CAUTION indicator
            //Serial.print(F("Caution: "));
            //Serial.println(out);
            brightness = out * 255.0;
            brightness = constrain(brightness, 0, 255);
            analogWrite(caution_LED_pin, brightness);
            break;
          case 3: // Zone 1 lighting
            //Serial.print(F("Zone 1 : "));
            //Serial.print(out);
            brightness = out * 255.0;
            brightness = constrain(brightness, 0, 255);
            analogWrite(zone1_lighting_pin, brightness);
            //Serial.print(F(" "));
            //Serial.println(brightness);
            break;
          case 4: // Zone 3 lighting
            //Serial.print(F("Zone 3 : "));
            //Serial.println(out);
            brightness = out * 255.0;
            brightness = constrain(brightness, 0, 255);
            analogWrite(zone3_lighting_pin, brightness);
            break;
          case 5: // vvi_status
            //Serial.print(F("vvi_status : "));
            //Serial.println(out);
            vvi_status = out; // Should be 2 or 0
            break;
          default:
            // The packet doesn't contain any indexes we handle.
            break;
        }
      }
    }
  }

  if (scan_matrix) {
    
    // The 1ms interrupt has set this flag. Time to scan the matrix...
    scan_matrix = false;
    
    // Iterate through the rows
    for (uint8_t i = 0; i < (sizeof rows / sizeof rows[0]); i++) {
      // Drive each row low one at a time
      pinMode(rows[i], OUTPUT);
      digitalWrite(rows[i], LOW);
      // Read all 16 bits of the MCP23017 to get the column states and
      // store the 16 bit value in the matrix array (we will ignore
      // the MSB). Bit is zero if button is pressed during scan, so invert
      // the value so that we have positive logic (bit = 1 for active).
      cur_matrix[i] = ~mcp.read();  // Returns (b << 8 | a)
      // Restore the row drive
      digitalWrite(rows[i], HIGH);
      pinMode(rows[i], INPUT_PULLUP);
    }

    // Matrix is scanned. Update the things that use it.

    // MCP Buttons

    // Warning/Caution button sends two commands as there are two
    // separate indicators, but only one physical button in the
    // Challenger 300. There are two indicator/button assemblies at
    // either end of the glare shield.
    // There are several buttons duplicated for Pilot/FO, but they are
    // unlikely to be pressed at the same time, so we don't care.
    p_warning_caution_clear.tick();
    fo_warning_caution_clear.tick();
    if (p_warning_caution_clear.delta == 1 || fo_warning_caution_clear.delta == 1) {
      Serial.println(F("MCP P or FO button pressed"));
      sendCommand(PSTR("sim/annunciator/clear_master_warning"));
      sendCommand(PSTR("sim/annunciator/clear_master_caution"));
    }
    else if (p_warning_caution_clear.delta == -1 || fo_warning_caution_clear.delta == -1)
      Serial.println(F("MCP P or FO button released"));


    // FGP buttons

    // Buttons/knobs are handled in groups from left to right.
    
    // There are two FD buttons at either end of the FGP
    fgp_p_fd.tick();
    fgp_fo_fd.tick();
    if (fgp_p_fd.delta == 1 || fgp_fo_fd.delta == 1) {
      Serial.println(F("FGP P or FO FD pressed"));
        sendCommand(PSTR("sim/autopilot/fdir_toggle"));
    }

    // Turning both CRS knobs will only cause one click to be sent to sim
    fgp_p_crs.tick();
    fgp_fo_crs.tick();    
    if (fgp_p_crs.delta == 1 || fgp_fo_crs.delta == 1) {
      Serial.println(F("FGP P or FO CRS +ve"));
      sendCommand(PSTR("sim/radios/obs_HSI_up"));
    }
    else if (fgp_p_crs.delta == -1 || fgp_fo_crs.delta == -1) {
      Serial.println(F("FGP P or FO CRS -ve"));
      sendCommand(PSTR("sim/radios/obs_HSI_down"));
    }

    // CRS rotary pushbutton change dataref on push, change back on release
    fgp_p_push_direct.tick();
    fgp_fo_push_direct.tick();
    if (fgp_p_push_direct.delta == 1 || fgp_fo_push_direct.delta == 1) {
      Serial.println(F("FGP P or FO PUSH DIRECT pressed"));
      writeDataref(PSTR("cl300/crc_butt"),1);
    }
    else if (fgp_p_push_direct.delta == -1 || fgp_fo_push_direct.delta == -1) {
      Serial.println(F("FGP P or FO PUSH DIRECT released"));
      writeDataref(PSTR("cl300/crc_butt"),0);
    }

    fgp_nav.tick();
    if (fgp_nav.delta == 1) {
      Serial.println(F("FGP NAV pressed"));
      sendCommand(PSTR("sim/autopilot/NAV"));
    }

    fgp_btn_hdg.tick();
    if (fgp_btn_hdg.delta == 1) {
      Serial.println(F("FGP HDG pressed"));
      sendCommand(PSTR("sim/autopilot/heading"));
    }

    fgp_appr.tick();
    if (fgp_appr.delta == 1) {
      Serial.println(F("FGP APPR pressed"));
      sendCommand(PSTR("sim/autopilot/approach"));
    }

    fgp_half_bank.tick();
    if (fgp_half_bank.delta == 1) {
      Serial.println(F("FGP 1/2 BANK pressed"));
      half_bank_value = !half_bank_value;
      writeDataref(PSTR("cl300/half_bank_h"),half_bank_value?1:0);
    }

    fgp_push_sync.tick();
    if (fgp_push_sync.delta == 1) {
      Serial.println(F("FGP PUSH SYNC pressed"));
      writeDataref(PSTR("cl300/fgp_s_hdg"),1);
    }
    else if (fgp_push_sync.delta == -1) {
      Serial.println(F("FGP PUSH SYNC released"));
      writeDataref(PSTR("cl300/fgp_s_hdg"),0);
    }

    fgp_hdg.tick();
    if (fgp_hdg.delta == 1) {
      Serial.println(F("FGP HDG +ve"));
      sendCommand(PSTR("sim/autopilot/heading_up"));
    }
    else if (fgp_hdg.delta == -1) {
      Serial.println(F("FGP HDG -ve"));
      sendCommand(PSTR("sim/autopilot/heading_down"));
    }

    fgp_b_c.tick();
    if (fgp_b_c.delta == 1) {
      Serial.println(F("FGP B/C pressed"));
      sendCommand(PSTR("sim/autopilot/back_course"));
    }


    fgp_flc.tick();
    if (fgp_flc.delta == 1) {
      Serial.println(F("FGP FLC pressed"));
      sendCommand(PSTR("sim/autopilot/level_change"));
      sendCommand(PSTR("sim/autopilot/speed_hold"));
    }    
    
    fgp_push_ias_mach.tick();
    if (fgp_push_ias_mach.delta == 1) {
      Serial.println(F("FGP IAS/MACH pressed"));
      sendCommand(PSTR("sim/autopilot/knots_mach_toggle"));
    }

    fgp_speed.tick();
    if (fgp_speed.delta == 1) {
      Serial.println(F("FGP SPEED +ve"));
      sendCommand(PSTR("sim/autopilot/airspeed_up"));
    }
    else if (fgp_speed.delta == -1) {
      Serial.println(F("FGP SPEED -ve"));
      sendCommand(PSTR("sim/autopilot/airspeed_down"));
    }

    fgp_vs.tick();
    if (fgp_vs.delta == 1) {
      Serial.println(F("FGP VS pressed"));
      sendCommand(PSTR("sim/autopilot/vertical_speed"));
    }

    fgp_vnav.tick();
    if (fgp_vnav.delta == 1) {
      Serial.println(F("FGP VNAV pressed"));
      sendCommand(PSTR("sim/autopilot/FMS"));
    }


    fgp_pitch.tick();
    if (fgp_pitch.delta == 1) {
      if (vvi_status){
        // vvi_status is not zero
        Serial.println(F("FGP Pitch Wheel UP. vvi_status not zero (VS)."));
        sendCommand(PSTR("sim/autopilot/vertical_speed_up"));
      }
      else
      {
        // vvi_status is zero
        Serial.println(F("FGP Pitch Wheel UP. vvi_status is zero (PTCH)."));
        sendCommand(PSTR("sim/autopilot/nose_up_pitch_mode"));
      }
    }
    else if (fgp_pitch.delta == -1) {
      if (vvi_status){
        // vvi_status is not zero
        Serial.println(F("FGP Pitch Wheel DOWN. vvi_status not zero (VS)."));
        sendCommand(PSTR("sim/autopilot/vertical_speed_down"));
      }
      else
      {
        // vvi_status is zero
        Serial.println(F("FGP Pitch Wheel DOWN. vvi_status is zero (PTCH)."));
        sendCommand(PSTR("sim/autopilot/nose_down_pitch_mode"));        
      }
    }


    fgp_btn_alt.tick();
    if (fgp_btn_alt.delta == 1) {
      Serial.println(F("FGP ALT pressed"));
      sendCommand(PSTR("sim/autopilot/altitude_hold"));
    }

    fgp_push_cancel.tick();
    if (fgp_push_cancel.delta == 1) {
      Serial.println(F("FGP PUSH CANCEL pressed (not present in sim)"));
    }
    
    fgp_alt.tick();
    if (fgp_alt.delta == 1) {
      Serial.println(F("FGP ALT +ve"));
      sendCommand(PSTR("cl300/autop/autop_alt_dial_up"));
    }
    else if (fgp_alt.delta == -1) {
      Serial.println(F("FGP ALT -ve"));
      sendCommand(PSTR("cl300/autop/autop_alt_dial_dn"));
    }

    fgp_ap.tick();
    if (fgp_ap.delta == 1) {
      Serial.println(F("FGP AP pressed"));
      sendCommand(PSTR("sim/autopilot/servos_toggle"));
      sendCommand(PSTR("sim/autopilot/fdir_servos_toggle"));
      sendCommand(PSTR("sim/autopilot/autothrottle_off"));
    }

    fgp_yd.tick();
    if (fgp_yd.delta == 1) {
      Serial.println(F("FGP YD pressed"));
      sendCommand(PSTR("sim/systems/yaw_damper_toggle"));
    }

    fgp_xfr.tick();
    if (fgp_xfr.delta == 1) {
      Serial.println(F("FGP XFR pressed"));
      xfr_value = !xfr_value;
      writeDataref(PSTR("cl300/autop_xfr_h"),xfr_value?1:0);
    }

    fgp_ap_yd_disc.tick();
    if (fgp_ap_yd_disc.delta == 1) {
      Serial.println(F("FGP AP/YD DISC pressed"));
      sendCommand(PSTR("sim/autopilot/servos_fdir_yawd_off"));
    }


  // DCP Encoders (Pilot and FO)

    dcp_p_tune_outer.tick();
    dcp_fo_tune_outer.tick();    
    if (dcp_p_tune_outer.delta == 1 || dcp_fo_tune_outer.delta == 1) {
      Serial.println(F("DCP TUNE COARSE +ve"));
      sendCommand(PSTR("xap/DCP/dcp_tune_right_coarse"));
    }
    else if (dcp_p_tune_outer.delta == -1 || dcp_fo_tune_outer.delta == -1) {
      Serial.println(F("DCP TUNE COARSE -ve"));
      sendCommand(PSTR("xap/DCP/dcp_tune_left_coarse"));
    }

    dcp_p_tune_inner.tick();
    dcp_fo_tune_inner.tick();    
    if (dcp_p_tune_inner.delta == 1 || dcp_fo_tune_inner.delta == 1) {
      Serial.println(F("DCP TUNE FINE +ve"));
      sendCommand(PSTR("xap/DCP/dcp_tune_right_fine"));
    }
    else if (dcp_p_tune_inner.delta == -1 || dcp_fo_tune_inner.delta == -1) {
      Serial.println(F("DCP TUNE FINE -ve"));
      sendCommand(PSTR("xap/DCP/dcp_tune_left_fine"));
    }

    dcp_p_menu.tick();
    dcp_fo_menu.tick();    
    if (dcp_p_menu.delta == 1 || dcp_fo_menu.delta == 1) {
      Serial.println(F("DCP MENU +ve"));
      sendCommand(PSTR("xap/DCP/dcp_menu_right"));
    }
    else if (dcp_p_menu.delta == -1 || dcp_fo_menu.delta == -1) {
      Serial.println(F("DCP MENU -ve"));
      sendCommand(PSTR("xap/DCP/dcp_menu_left"));
    }

    dcp_p_data.tick();
    dcp_fo_data.tick();    
    if (dcp_p_data.delta == 1 || dcp_fo_data.delta == 1) {
      Serial.println(F("DCP DATA +ve"));
      sendCommand(PSTR("xap/DCP/dcp_data_right"));
    }
    else if (dcp_p_data.delta == -1 || dcp_fo_data.delta == -1) {
      Serial.println(F("DCP DATA -ve"));
      sendCommand(PSTR("xap/DCP/dcp_data_left"));
    }

    dcp_p_tilt.tick();
    dcp_fo_tilt.tick();    
    if (dcp_p_tilt.delta == 1 || dcp_fo_tilt.delta == 1) {
      Serial.println(F("DCP TILT +ve"));
      tilt_value = tilt_value < 1500 ? tilt_value+1 : 0;
      writeDataref(PSTR("cl300/dcp_tilt"), tilt_value);
    }
    else if (dcp_p_tilt.delta == -1 || dcp_fo_tilt.delta == -1) {
      Serial.println(F("DCP TILT -ve"));
      tilt_value = tilt_value > -1500 ? tilt_value-1 : 0;
      writeDataref(PSTR("cl300/dcp_tilt"), tilt_value);
    }

    dcp_p_range.tick();
    dcp_fo_range.tick();
    if (dcp_p_range.delta == 1 || dcp_fo_range.delta == 1) {
      Serial.println(F("DCP RANGE +ve"));
      range_value = range_value < 150 ? range_value+1 : 0;
      writeDataref(PSTR("cl300/dcp_range"), range_value);
    }
    else if (dcp_p_range.delta == -1 || dcp_fo_range.delta == -1) {
      Serial.println(F("DCP RANGE -ve"));
      range_value = range_value > -150 ? range_value-1 : 0;
      writeDataref(PSTR("cl300/dcp_range"), range_value);
    }



  // DCP buttons

    dcp_p_1_2.tick();
    dcp_fo_1_2.tick();
    if (dcp_p_1_2.delta == 1 || dcp_fo_1_2.delta == 1) {
      Serial.println(F("DCP 1/2 pressed"));
      dcp_1_2_value = !dcp_1_2_value;
      writeDataref(PSTR("cl300/dcp_1_2"), dcp_1_2_value?1:0);
    }
    
    dcp_p_swap.tick();
    dcp_fo_swap.tick();
    if (dcp_p_swap.delta == 1 || dcp_fo_swap.delta == 1) {
      Serial.println(F("DCP SWAP pressed"));
      sendCommand(PSTR("xap/DCP/dcp_tune_stb"));
    }

    dcp_p_dme_h.tick();
    dcp_fo_dme_h.tick();
    if (dcp_p_dme_h.delta == 1 || dcp_fo_dme_h.delta == 1) {
      Serial.println(F("DCP DME H pressed"));
      dme_h_value = !dme_h_value;
      writeDataref(PSTR("cl300/dcp/dmeh"), dme_h_value?1:0);
    }

    dcp_p_radio.tick();
    dcp_fo_radio.tick();
    if (dcp_p_radio.delta == 1 || dcp_fo_radio.delta == 1) {
      Serial.println(F("DCP RADIO pressed"));
      radio_value = !radio_value;
      writeDataref(PSTR("cl300/mfd_sel_state_h"), radio_value?1:0);
    }

    dcp_p_nav_src.tick();
    dcp_fo_nav_src.tick();
    if (dcp_p_nav_src.delta == 1 || dcp_fo_nav_src.delta == 1) {
      Serial.println(F("DCP NAV SRC pressed"));
      sendCommand(PSTR("cl300/DCP/navsrc_toggle"));
    }

    dcp_p_brg_src.tick();
    dcp_fo_brg_src.tick();
    if (dcp_p_brg_src.delta == 1 || dcp_fo_brg_src.delta == 1) {
      Serial.println(F("DCP BRG SRC pressed"));
      sendCommand(PSTR("cl300/DCP/brgsrc_toggle"));
    }

    dcp_p_select.tick();
    dcp_fo_select.tick();
    if (dcp_p_select.delta == 1 || dcp_fo_select.delta == 1) {
      Serial.println(F("DCP SELECT pressed"));
      sendCommand(PSTR("cl300/DCP/dcp_data_sel"));
    }

    dcp_p_frmt.tick();
    dcp_fo_frmt.tick();
    if (dcp_p_frmt.delta == 1 || dcp_fo_frmt.delta == 1) {
      Serial.println(F("DCP FRMT pressed"));
      sendCommand(PSTR("sim/instruments/EFIS_mode_up"));
    }

    dcp_p_refs.tick();
    dcp_fo_refs.tick();
    if (dcp_p_refs.delta == 1 || dcp_fo_refs.delta == 1) {
      Serial.println(F("DCP REFS pressed"));
      sendCommand(PSTR("cl300/DCP/dcp_refs_button"));
    }


    dcp_p_tfc.tick();
    dcp_fo_tfc.tick();
    if (dcp_p_tfc.delta == 1 || dcp_fo_tfc.delta == 1) {
      Serial.println(F("DCP TFC pressed"));
      tfc_value = !tfc_value;
      writeDataref(PSTR("sim/cockpit/switches/EFIS_shows_tcas"), tfc_value?1:0);
    }

    dcp_p_radar.tick();
    dcp_fo_radar.tick();
    if (dcp_p_radar.delta == 1 || dcp_fo_radar.delta == 1) {
      Serial.println(F("DCP RADAR pressed"));
      radar_value = !radar_value;
      writeDataref(PSTR("cl300/scp_radar"), tfc_value?1:0);
    }

    dcp_p_tr_wx.tick();
    dcp_fo_tr_wx.tick();
    if (dcp_p_tr_wx.delta == 1 || dcp_fo_tr_wx.delta == 1) {
      Serial.println(F("DCP TR WX pressed"));
      sendCommand(PSTR("cl300/tr_wx"));
    }

  // fixme: action unknown for this button
    dcp_p_push_auto.tick();
    dcp_fo_push_auto.tick();
    if (dcp_p_push_auto.delta == 1 || dcp_fo_push_auto.delta == 1) {
      Serial.println(F("DCP PUSH AUTO pressed (fixme: unknown action)"));
      //sendCommand(PSTR(""));
    }

  } // if (scan_matrix)

  // Testing code here

} // loop()
