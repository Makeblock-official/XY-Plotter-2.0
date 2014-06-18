// GRemoteCNC - mini host interfacing with Arduino GCode Interpreter

// Setup, event loops and state

import controlP5.*;
import processing.serial.*;
import java.util.StringTokenizer;
import java.awt.event.KeyEvent;

// Global variables
ControlP5 cP5;
Serial port = null;
int baudrate = 115200;
EnumXYZ idx = new EnumXYZ();

int resolution = 100000; // of a unit, for fixed-point calculations;
int position[]; // absolute
int jog[]; // relative
int accumulated_jog[]; // relative

int memory[]; // absolute
boolean memorySet[];

float feed[];
float feedInc = 1*250; // mm
float lastFeed = 0;

float homing_limit[]; // absolute
float homingInfinity = 100*250; // mm
float homingFeed = 10*250; // mm/min

float arc_radius = 5; // current units
float arc_start = 0; // degrees
float arc_end = 360; // degrees
boolean ArcCCW = false; // direction

int i;
float f;

// State flags
boolean PortWasReset = false;
boolean PortResponding = false;
boolean WaitingForResponse = false;
boolean HaveStringToSend = false;

boolean SendingSequence = false;
boolean Paused = false;
boolean SequenceLastLineSent = false;

boolean G_AbsoluteMode = true;
boolean G_InchMode = false;

boolean RapidPositioning = true;
boolean FractionalJog = false;
boolean HomingSetZero = true;

boolean UI_ReloadJogDDL = false;
boolean UI_ReloadArcTF = false;
boolean UI_ClearGCodeTF = false;
boolean UI_ClearFocusTF = false;

boolean ArcMode = false;
boolean XYZMode = false;
boolean ZeroMode = false;
boolean MemMode = false;

// setup
void setup() {
  size(400,700);
  smooth();
  
  cP5 = new ControlP5(this);
  cP5.setColorForeground(0xff00aa00);
  cP5.setColorBackground(0xff006600);
  cP5.setColorLabel(0xffdddddd);
  cP5.setColorValue(0xff88ff88);
  cP5.setColorActive(0xff00ff00);
  ControlFont font = new ControlFont();
  font.setFontSize(28);
  cP5.setFont(font);
  position = new int[idx.size()];
  jog = new int[idx.size()];
  accumulated_jog = new int[idx.size()];
  memory = new int[idx.size()];
  memorySet = new boolean[idx.size()];
  feed = new float[idx.size()];
  homing_limit = new float[idx.size()];
  
  // init
  for (int i=0; i<idx.size(); i++) {
    position[i] = 0;
    jog[i] = resolution;
    accumulated_jog[i] = 0;
    memorySet[i] = false;
    feed[i] = 10*2500;  // mm/min
    homing_limit[i] = 0;
  }

  setup_console(10,35,150,150);  
  setup_func_buttons(10,185);
  setup_toggles(10,218);
  setup_jog_buttons(10,440);
  
  setup_jog_controls(10,295,50); 
  setup_homing_controls(10,305,40); 
  setup_arc_controls(10,315,30); 
  
  setup_port_selector(30,25,180,130); 
  setup_port_led(10,10);
    
}

// draw loop
void draw() { 
  background(0x00000000);  // background
  update_console();
  update_port_led();
  update_toggles(); 
  update_func_buttons(); 
  update_jog_buttons();
  update_jog_controls();
  update_homing_controls();
  update_port_selector();
  update_arc_controls();
  update_textfields(); // manage textfields focus
  update_groups(); // manage groups
}

// cP5 UI events
void controlEvent(ControlEvent theEvent) {
  boolean RenderZero = false;
  String s = "";
  int arc_offset;
  int j[] = new int[idx.size()];
  for (int i=0; i<idx.size(); i++) j[i] = 0;  // clear temp array

  // because only permitted (for current state) elements are available in the UI,
  // we assume that if we received an event, we can handle it without checking state
  // the above is not true for keypresses, where state has to be explicitly checked before handling event
    
  if(theEvent.isGroup()) { 

    
    // baud rate selected
    if(theEvent.group().name()=="BAUD") { baudrate = (int)theEvent.group().value(); println("baud="+baudrate); return; }

    // serial port selected
    if(theEvent.group().name()=="PORT") {
      if (port != null) port.stop();

      try { port = new Serial(this, Serial.list()[(int)theEvent.group().value()], baudrate); }
      catch (Exception e) { console_println(": can't open selected port!"); }
      clear_console();

      PortWasReset = true;
      PortResponding = false;
      WaitingForResponse = false;
      println("port open: " + Serial.list()[(int)theEvent.group().value()]);

      port.bufferUntil('\n');
      port.write("\r\n");
      return;
    }

    // jog setting selected
    if(theEvent.group().name()=="JOG X") {
      i = (int)theEvent.group().value();
      jog[idx.X] = intCoord(FractionalJog ? jog_frac_value[i] : jog_dec_value[i]);
//      println(jog[idx.X]);
      // store current jog dropdown values - workaround to enable inc/dec of jog values with keyboard shortcut (dropdownlist doesn't seem to have .setValue)
      jog_ddl_idx[idx.X] = i; jog_ddl_frac[idx.X] = FractionalJog;
    }
    if(theEvent.group().name()=="JOG Y") {
      i = (int)theEvent.group().value();
      jog[idx.Y] = intCoord(FractionalJog ? jog_frac_value[i] : jog_dec_value[i]);
//      println(jog[idx.Y]);
      // store current jog dropdown values - workaround to enable inc/dec of jog values with keyboard shortcut (dropdownlist doesn't seem to have .setValue)
      jog_ddl_idx[idx.Y] = i; jog_ddl_frac[idx.Y] = FractionalJog;
    }
    if(theEvent.group().name()=="JOG Z") {
      i = (int)theEvent.group().value();
      jog[idx.Z] = intCoord(FractionalJog ? jog_frac_value[i] : jog_dec_value[i]);
//      println(jog[idx.Z]);
      // store current jog dropdown values - workaround to enable inc/dec of jog values with keyboard shortcut (dropdownlist doesn't seem to have .setValue)
      jog_ddl_idx[idx.Z] = i; jog_ddl_frac[idx.Z] = FractionalJog;
    }

  }
  
  if(theEvent.isController()) { 
   
//    print("control event from controller: "+theEvent.controller().name());
//    println(", value : "+theEvent.controller().value());

    // manually entered command
    if(theEvent.controller().name() == "GCODE") {
      s = theEvent.controller().stringValue().toUpperCase();
      HaveStringToSend = true; // UI_ClearFocusTF = true;
    }
    
    // absolute mode toggle
    if(theEvent.controller().name() == "absolute_mode") {
      s = ((int)theEvent.controller().value() == 1) ? "G90" : "G91";
      HaveStringToSend = true;
    }

    // inch mode toggle
    if(theEvent.controller().name() == "inch_mode") {
      s = ((int)theEvent.controller().value() == 1) ? "G20" : "G21";
      HaveStringToSend = true;
    }

    // send file button
    if(theEvent.controller().name() == "SEND FILE") {
        selectInput("Select GCode file to send","send_file");
//      String file = selectInput("Select GCode file to send");
//      if (file == null) return;
//      send_file(file);
    }
        
    // cancel (sending file) button
    if(theEvent.controller().name() == "CANCEL") {
      SendingSequence = false;
      Paused = false;
      println(": send sequence cancelled"); console_println(": send sequence cancelled");
    }

    // pause/resume (sending file) button
    if(theEvent.controller().name() == "PAUSE/RESUME") {
      if (!Paused) {
        println(": send sequence paused"); console_println(": send sequence paused");
        Paused = true;
      } else {
        println(": send sequence resumed"); console_println(": send sequence resumed");
        Paused = false;
        send_next_line();
      }
    }

    // jog control toggles
    if(theEvent.controller().name() == "fractional_jog") {
      if ((int)theEvent.controller().value() == 1) FractionalJog = true;
      else FractionalJog = false;
      UI_ReloadJogDDL = true;
    }
    if(theEvent.controller().name() == "rapid_positioning") {
      if ((int)theEvent.controller().value() == 1) RapidPositioning = true;
      else { RapidPositioning = false; lastFeed = 0; }
    }
    
    if(theEvent.controller().name() == "FEED X") { feed[idx.X] = theEvent.controller().value(); }
    if(theEvent.controller().name() == "FEED Y") { feed[idx.Y] = theEvent.controller().value(); }
    if(theEvent.controller().name() == "FEED Z") { feed[idx.Z] = theEvent.controller().value(); }

    if(theEvent.controller().name() == "arc_mode") {
      if (XYZMode || ZeroMode || MemMode) return; //UI should be locked instead of state check
      if ((int)theEvent.controller().value() == 1) {
        ArcMode = true;
      }
      else {
        ArcMode = false;
      }
      open_group(ArcMode? 'A':'J');
    }
    
    if(theEvent.controller().name() == "xyz_mode") {
      if (ArcMode || ZeroMode || MemMode) return; //UI should be locked instead of state check
      if ((int)theEvent.controller().value() == 1) {
        for (i=0; i<accumulated_jog.length; i++) accumulated_jog[i] = 0;        
        XYZMode = true;
      }
      else {
        int jog_total = 0;
        for (i=0; i<accumulated_jog.length; i++) jog_total += abs(accumulated_jog[i]);
        if (jog_total > 0) {
          s = jog_string(accumulated_jog, G_AbsoluteMode, false);
          HaveStringToSend = true;  
        }
        XYZMode = false;
        UI_ClearGCodeTF = true;
      }
    }

    if(theEvent.controller().name() == "zero_mode") {
      if (ArcMode || XYZMode || MemMode) return; //UI should be locked instead of state check
      if ((int)theEvent.controller().value() == 1) {
        ZeroMode = true;
      }
      else {
        ZeroMode = false;
        UI_ClearGCodeTF = true;
      }
    }

    if(theEvent.controller().name() == "mem_mode") {
      if (ArcMode || XYZMode || ZeroMode) return; //UI should be locked instead of state check
      if ((int)theEvent.controller().value() == 1) {
        MemMode = true;
      }
      else {
        MemMode = false;
        UI_ClearGCodeTF = true;
      }
    }
    
    // homing controls
    if(theEvent.controller().name() == "homing_set_zero") {
      if ((int)theEvent.controller().value() == 1) HomingSetZero = true;
      else HomingSetZero = false;
    }
//    println("raw: "+homing_limit[idx.X] + ", ceil: "+ceil(100*homing_limit[idx.X]) + ", floor: "+floor(100*homing_limit[idx.X]) );
    if(theEvent.controller().name() == "homing_limit_x") homing_limit[idx.X] = theEvent.controller().value();
    if(theEvent.controller().name() == "homing_limit_y") homing_limit[idx.Y] = theEvent.controller().value();
    if(theEvent.controller().name() == "homing_limit_z") homing_limit[idx.Z] = theEvent.controller().value();
    // fix for values returned by controller - they often have many decimals (accumulated errors from float/double additions/subtractions?)
    for (i = 0; i<idx.size(); i++) homing_limit[i] = (homing_limit[i]>=0)? floor(homing_limit[i]*100)/100.0 : ceil(homing_limit[i]*100)/100.0;
    if(theEvent.controller().name() == "homing_infinity") homingInfinity = theEvent.controller().value();
    if(theEvent.controller().name() == "homing_feed") homingFeed = theEvent.controller().value();
    // home XY & Z buttons
    if(theEvent.controller().name() == "HOME XY") {
      homing_sequence("XY");
    }
    if(theEvent.controller().name() == "HOME Z") {
      homing_sequence("Z");
    }    

    // arc controls
    if(theEvent.controller().name() == "ARC_CCW") {
      if ((int)theEvent.controller().value() == 1) ArcCCW = true;
      else ArcCCW = false;
    }
    if(theEvent.controller().name() == "ARC_RADIUS") {
      f = arc_radius;
      arc_radius = isFloat(theEvent.controller().stringValue()) ? Float.parseFloat(theEvent.controller().stringValue()) : arc_radius; 
      if (arc_radius <=0) arc_radius = f;
      UI_ReloadArcTF = true;
      UI_ClearFocusTF = true;
    }
    if(theEvent.controller().name() == "ARC_START") {
      f = arc_start;
      arc_start = isFloat(theEvent.controller().stringValue()) ? Float.parseFloat(theEvent.controller().stringValue()) : arc_start; 
      if (arc_start <0 || arc_start >= 360 || arc_start >= arc_end) arc_start = f;
      UI_ReloadArcTF = true;
      UI_ClearFocusTF = true;
    }
    if(theEvent.controller().name() == "ARC_END") { 
      f = arc_end;
      arc_end = isFloat(theEvent.controller().stringValue()) ? Float.parseFloat(theEvent.controller().stringValue()) : arc_end; 
      if (arc_end <=0 || arc_end > 360 || arc_start >= arc_end) arc_end = f;
      UI_ReloadArcTF = true; 
      UI_ClearFocusTF = true;
    }

    // jog button events
    if (theEvent.controller().name() == "X+") { 
      j[idx.X] = jog[idx.X];
      if (XYZMode) { accumulated_jog[idx.X] += jog[idx.X]; return; }
      if (ZeroMode) j[idx.X] = -position[idx.X];
      if (MemMode) 
        if (memorySet[idx.X]) j[idx.X] = memory[idx.X] - position[idx.X];
        else return;
      if (ArcMode) {
        arc_offset = ArcCCW ? -90 : 90;
        s = arc_string(ArcCCW, arc_radius, arc_offset + (ArcCCW? arc_start : -arc_start), arc_offset + (ArcCCW? arc_end : -arc_end), G_AbsoluteMode);
        HaveStringToSend = true;
      } else if (j[idx.X] != 0) { s = jog_string(j, G_AbsoluteMode, RenderZero); HaveStringToSend = true; }
      if (ZeroMode) { ZeroMode = false; UI_ClearGCodeTF = true; }
      if (MemMode) { MemMode = false; UI_ClearGCodeTF = true; }
    }
    if (theEvent.controller().name() == "X-") { 
      j[idx.X] = -jog[idx.X];
      if (XYZMode) { accumulated_jog[idx.X] -= jog[idx.X]; return; }
      if (MemMode) { memory[idx.X] = position[idx.X]; memorySet[idx.X] = true; return; }
      if (ArcMode) {
        arc_offset = ArcCCW ? 90 : -90;
        s = arc_string(ArcCCW, arc_radius, arc_offset + (ArcCCW? arc_start : -arc_start), arc_offset + (ArcCCW? arc_end : -arc_end), G_AbsoluteMode);
        HaveStringToSend = true;
      } else {
        s = ZeroMode ? "G92 X0" : jog_string(j, G_AbsoluteMode, RenderZero);
        HaveStringToSend = true; 
      }
//      if (ZeroMode) { ZeroMode = false; UI_ClearGCodeTF = true; }
    }
    if (theEvent.controller().name() == "Y+") { 
      j[idx.Y] = jog[idx.Y];
      if (XYZMode) { accumulated_jog[idx.Y] += jog[idx.Y]; return; }
      if (ZeroMode) j[idx.Y] = -position[idx.Y];
      if (MemMode) 
        if (memorySet[idx.Y]) j[idx.Y] = memory[idx.Y] - position[idx.Y];
        else return;
      if (ArcMode) {
        arc_offset = ArcCCW ? 0 : 180;
        s = arc_string(ArcCCW, arc_radius, arc_offset + (ArcCCW? arc_start : -arc_start), arc_offset + (ArcCCW? arc_end : -arc_end), G_AbsoluteMode);
        HaveStringToSend = true;
      } else if (j[idx.Y] != 0) { s = jog_string(j, G_AbsoluteMode, RenderZero); HaveStringToSend = true; }
      if (ZeroMode) { ZeroMode = false; UI_ClearGCodeTF = true; }
      if (MemMode) { MemMode = false; UI_ClearGCodeTF = true; }
    }
    if (theEvent.controller().name() == "Y-") { 
      j[idx.Y] = -jog[idx.Y];
      if (XYZMode) { accumulated_jog[idx.Y] -= jog[idx.Y]; return; }
      if (MemMode) { memory[idx.Y] = position[idx.Y]; memorySet[idx.Y] = true; return; }
      if (ArcMode) {
        arc_offset = ArcCCW ? -180 : 0;
        s = arc_string(ArcCCW, arc_radius, arc_offset + (ArcCCW? arc_start : -arc_start), arc_offset + (ArcCCW? arc_end : -arc_end), G_AbsoluteMode);
        HaveStringToSend = true;
      } else {
        s = ZeroMode ? "G92 Y0" : jog_string(j, G_AbsoluteMode, RenderZero);
        HaveStringToSend = true; 
      }
//      if (ZeroMode) { ZeroMode = false; UI_ClearGCodeTF = true; }
    }
    if (theEvent.controller().name() == "Z+") { 
      j[idx.Z] = jog[idx.Z];
      if (XYZMode) { accumulated_jog[idx.Z] += jog[idx.Z]; return; }
      if (ZeroMode) j[idx.Z] = -position[idx.Z];
      if (MemMode) 
        if (memorySet[idx.Z]) j[idx.Z] = memory[idx.Z] - position[idx.Z];
        else return;
      if (j[idx.Z] != 0) { s = jog_string(j, G_AbsoluteMode, RenderZero); HaveStringToSend = true; }

      if (ZeroMode) { ZeroMode = false; UI_ClearGCodeTF = true; }
      if (MemMode) { MemMode = false; UI_ClearGCodeTF = true; }
    }
    if (theEvent.controller().name() == "Z-") { 
      j[idx.Z] = -jog[idx.Z];
      if (XYZMode) { accumulated_jog[idx.Z] -= jog[idx.Z]; return; }
      if (MemMode) { memory[idx.Z] = position[idx.Z]; memorySet[idx.Z] = true; return; }
      s = ZeroMode ? "G92 Z0" : jog_string(j, G_AbsoluteMode, RenderZero);
      HaveStringToSend = true; 
//      if (ZeroMode) { ZeroMode = false; UI_ClearGCodeTF = true; }
    }
    
    if (HaveStringToSend) {
      HaveStringToSend = false;
      if (WaitingForResponse) { delay(50); } // wait a bit
      if (WaitingForResponse) { java.awt.Toolkit.getDefaultToolkit().beep(); return; } // beep & exit if still no response from port
      process_command(s);
    }
  }  
}

// keyboard events

void keyPressed() {
  String s = "";
  if (!(PortResponding && (!SendingSequence || SendingSequence && Paused))) return;
  if (((Textfield)cP5.controller("GCODE")).isFocus()) return; // do not process keystrokes while editing GCode
  
  if (key == 'j' || key == 'J') open_group('J');
  if (key == 'h' || key == 'H') open_group('H');
  if (key == 'a' || key == 'A') open_group('A');

  if (key == '+') { feed[idx.X] += feedInc; feed[idx.Y] += feedInc; }
  if (key == '-') { feed[idx.X] -= feedInc; feed[idx.Y] -= feedInc; }
  
  if (Jogging_grp.isOpen()) {
    if (key == 'x')
      if (jog_ddl_idx[idx.X] > 0) {
        jog_ddl_idx[idx.X] -= 1;
        jog_ddl_frac[idx.X] = FractionalJog;
        jog[idx.X] = intCoord(jog_ddl_frac[idx.X] ? jog_frac_value[jog_ddl_idx[idx.X]] : jog_dec_value[jog_ddl_idx[idx.X]]);
//        println(jog[idx.X]);
      }
    if (key == 'X')
      if (jog_ddl_idx[idx.X] < min(jog_frac_name.length, jog_dec_name.length)-1) {
        jog_ddl_idx[idx.X] += 1;
        jog_ddl_frac[idx.X] = FractionalJog;
        jog[idx.X] = intCoord(jog_ddl_frac[idx.X] ? jog_frac_value[jog_ddl_idx[idx.X]] : jog_dec_value[jog_ddl_idx[idx.X]]);
//        println(jog[idx.X]);
      }
      
    if (key == 'y')
      if (jog_ddl_idx[idx.Y] > 0) {
        jog_ddl_idx[idx.Y] -= 1;
        jog_ddl_frac[idx.Y] = FractionalJog;
        jog[idx.Y] = intCoord(jog_ddl_frac[idx.Y] ? jog_frac_value[jog_ddl_idx[idx.Y]] : jog_dec_value[jog_ddl_idx[idx.Y]]);
//        println(jog[idx.Y]);
      }
    if (key == 'Y')
      if (jog_ddl_idx[idx.Y] < min(jog_frac_name.length, jog_dec_name.length)-1) {
        jog_ddl_idx[idx.Y] += 1;
        jog_ddl_frac[idx.Y] = FractionalJog;
        jog[idx.Y] = intCoord(jog_ddl_frac[idx.Y] ? jog_frac_value[jog_ddl_idx[idx.Y]] : jog_dec_value[jog_ddl_idx[idx.Y]]);
//        println(jog[idx.Y]);
      }
      
    if (key == 'z')
      if (jog_ddl_idx[idx.Z] > 0) {
        jog_ddl_idx[idx.Z] -= 1;
        jog_ddl_frac[idx.Z] = FractionalJog;
        jog[idx.Z] = intCoord(jog_ddl_frac[idx.Z] ? jog_frac_value[jog_ddl_idx[idx.Z]] : jog_dec_value[jog_ddl_idx[idx.Z]]);
//        println(jog[idx.Z]);
      }
    if (key == 'Z')
      if (jog_ddl_idx[idx.Z] < min(jog_frac_name.length, jog_dec_name.length)-1) {
        jog_ddl_idx[idx.Z] += 1;
        jog_ddl_frac[idx.Z] = FractionalJog;
        jog[idx.Z] = intCoord(jog_ddl_frac[idx.Z] ? jog_frac_value[jog_ddl_idx[idx.Z]] : jog_dec_value[jog_ddl_idx[idx.Z]]);
//        println(jog[idx.Z]);
      }
      
    if (key == 'f' || key == 'F') {
      ((Toggle)cP5.controller("fractional_jog")).setValue(!FractionalJog);
    }
    if (key == 'r' || key == 'R') {
      ((Toggle)cP5.controller("rapid_positioning")).setValue(!RapidPositioning);
    }
  }

  if (key == CODED && keyCode == RIGHT) { 
    ((Button)cP5.controller("X+")).setValue(ControlP5.PRESSED);
  }
  if (key == CODED && keyCode == LEFT) { 
    ((Button)cP5.controller("X-")).setValue(ControlP5.PRESSED);
  }
  if (key == CODED && keyCode == UP) { 
    ((Button)cP5.controller("Y+")).setValue(ControlP5.PRESSED);
  }
  if (key == CODED && keyCode == DOWN) { 
    ((Button)cP5.controller("Y-")).setValue(ControlP5.PRESSED);
  }
  if (key == CODED && keyCode == KeyEvent.VK_PAGE_UP) { 
    ((Button)cP5.controller("Z+")).setValue(ControlP5.PRESSED);
  }
  if (key == CODED && keyCode == KeyEvent.VK_PAGE_DOWN) { 
    ((Button)cP5.controller("Z-")).setValue(ControlP5.PRESSED);
  }  
  if (key == DELETE) {
    ((Toggle)cP5.controller("xyz_mode")).setValue(!XYZMode);
  }
  if (key == CODED && keyCode == KeyEvent.VK_INSERT) {
    ((Toggle)cP5.controller("arc_mode")).setValue(!ArcMode); open_group(ArcMode? 'A':'J');
  }
  if (key == CODED && keyCode == KeyEvent.VK_HOME) {
    ((Toggle)cP5.controller("zero_mode")).setValue(!ZeroMode);
  }
  if (key == CODED && keyCode == KeyEvent.VK_END) {
    ((Toggle)cP5.controller("mem_mode")).setValue(!MemMode);
  }
}

// serial events
void serialEvent(Serial port)
{
//  if(!SendingSequence) delay(100);
  delay(10); String s = port.readString();
  println("=> "+s.trim()); console_println("> "+s.trim());
  s = s.trim().toUpperCase();
  
  // process response
  // start/ok line-based protocol is supported by most reprap firmware
  
  // firmware reset (not all firmware does this though)
  if (s.equals("START")) {
    println("firmware start, sending init sequence");
    SendingSequence = false; Paused = false;
    PortWasReset = false; PortResponding = true; WaitingForResponse = false;
    for (int i=0; i<idx.size(); i++) position[i] = 0; // zero position
    init_sequence();
    return;
  }

  // response to a command
  if (s.equals("OK")) { 
    WaitingForResponse = false; // let everyone know they can send more to the port
    if (SendingSequence && !Paused) send_next_line();
    if (SendingSequence && SequenceLastLineSent) { 
      SequenceLastLineSent = false;
      SendingSequence = false;
      println(": done sending sequence"); console_println(": done sending sequence");
    }
    return;
  }

  // if we received something other than OK or START after port reset, assume port is responding
  // kludgy but should work for firmware that supports 'ok' but not 'start' e.g. Grbl (as of 0.7d)
  if (PortWasReset) {
    println("port reset, sending init sequence");
    PortWasReset = false; PortResponding = true; init_sequence();
  }
}

