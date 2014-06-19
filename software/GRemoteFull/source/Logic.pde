// business logic

// Storage for sending a file - should be a class containing GCodeCommand[]?
String[] gcode_sequence;
int next_gcode_line = 0;

class GCodeCommand
{
  EnumAZ idx = new EnumAZ();
  public int G = (int)'G';
  public int M = (int)'M';
  public int NOP = (int)' ';
  public int command = 0;
  public int code = 0;
  
  public boolean has[];
  public float value[];
  
  public GCodeCommand() { 
    init();
  }

  public GCodeCommand(String s) { 
    init();
    parse(s);
  }
  
  public GCodeCommand(char command, int code) { 
    init();
    if (command == 'G' || command == 'M') { this.command = (int)command; this.code = code; }
  }
  
  void init() {
    command = this.NOP;
    has = new boolean[this.idx.size()]; value = new float[this.idx.size()];
    for(int i=0; i<has.length; i++) this.has[i] = false;
  }

  void set(int idx, float value) {
    this.value[idx] = value;
    this.has[idx] = true;
  }
  
  public String render() {
    String out = "";
    int renderFirst[] = { idx.X, idx.Y, idx.Z, idx.I, idx.J, idx.K, idx.F };
    boolean RenderedYet[] = new boolean[idx.size()];
    for(int i=0; i<RenderedYet.length; i++) RenderedYet[i] = false;
    
    if (this.command == this.G) { out = "G" + this.code; RenderedYet[idx.G] = true; }
    if (this.command == this.M) { out = "M" + this.code; RenderedYet[idx.M] = true; }
    
    for (int i=0; i<renderFirst.length; i++) {
      if (this.has[renderFirst[i]]) {
        out += (" " + idx.strVal[renderFirst[i]] + this.value[renderFirst[i]]);
        RenderedYet[renderFirst[i]] = true;
      }
    }
    for (int i=0; i<RenderedYet.length; i++) {
      if (this.has[i] && !RenderedYet[i]) {
        out += (" " + idx.strVal[i] + this.value[i]);
        RenderedYet[i] = true;
      }
    }    
    return out;
  }
  
  void parse(String cmd) {
    StringTokenizer st = new StringTokenizer(cmd);
    String token = ""; 
    while(st.hasMoreTokens()) {
      token = st.nextToken();
//      println(token);
      if (token.substring(0,1).toUpperCase().equals("G")) {
        this.command = (int)'G'; this.code = Integer.parseInt(token.substring(1));
      }
      if (token.substring(0,1).toUpperCase().equals("M")) {
        this.command = (int)'M'; this.code = Integer.parseInt(token.substring(1));
      }
      // there could be other commands e.g. F10 set feedrate 
      if (this.command != this.NOP)
        for(int i=0; i<this.idx.size(); i++)
          if (token.substring(0,1).toUpperCase().equals(this.idx.strVal[i])) { this.has[i] = true; this.value[i] = Float.parseFloat(token.substring(1)); }
    }
  }

}

// Logic related functions
//

// file sending handler
//void send_file(String f) {
void send_file(File f) {
  if (f == null) return;
  gcode_sequence = loadStrings(f);
  if (gcode_sequence == null) {
    println(": unable to open the file"); console_println(": unable to open the file");
    return;
  }
  // set up state and send first line
  SendingSequence = true;
  next_gcode_line = 0;
  send_next_line();
}

// send next line of gcode_sequence indexed by next_gcode_line
// subsequent send_next_line() will be called by eventSerial upon receiving 'ok' response and looking at the state
void send_next_line() {

  // check state
  if (!SendingSequence || WaitingForResponse) return;
 
  // check EOF and skip empty lines
  while (true) {
    if (gcode_sequence.length == next_gcode_line) { // EOF
      SequenceLastLineSent = true; 
//      SendingSequence = false;
//      println("Done sending sequence"); console_println("Done sending sequence");
      return; 
    } 
    if (gcode_sequence[next_gcode_line].trim().equals("")) next_gcode_line++; // skip empty
    else break;
  }
  // process command
  String s = gcode_sequence[next_gcode_line].trim().toUpperCase();
  process_command(s);
//  if (proceed) { port.write(s); println("<= "+s); console_println("< "+s); }
  next_gcode_line++;
}


// process command to be sent to firmware and adjust state accordingly
// return true to proceed sending command to firmware, false otherwise
// if false is returned, command will be lost unless logic is somehow stores it to send at a later time
void process_command(String s) {
  GCodeCommand cmd = new GCodeCommand(s);
  EnumAZ idx = new EnumAZ();
  EnumXYZ xyz = new EnumXYZ();
  //println("cmd: " + cmd.action + cmd.code);

  if (cmd.command == cmd.G) {
    switch (cmd.code) {
      // Parse GCode and adjust current position (limited by current resolution)
      case 0:
      case 1:
      case 2:
      case 3:
        for (int i=0; i<xyz.size(); i++)
          if (cmd.has[xyz.toAZ(xyz.X+i)]) position[xyz.X+i] = G_AbsoluteMode? intCoord(cmd.value[xyz.toAZ(xyz.X+i)]) : (position[xyz.X+i] + intCoord(cmd.value[xyz.toAZ(xyz.X+i)]));
        break;        

      // Intercept absolute and inch mode commands and change state
      // Return false to avoid duplicate command from toggles triggered by state change, unless state doesn't change
      case 20:
      case 21:
        // set waiting state early to avoid triggering UI event and duplicate command
        WaitingForResponse = true;
        boolean Inches = G_InchMode;
        G_InchMode = (cmd.code == 20)? true : false;
        if (Inches != G_InchMode) {
          if (G_InchMode) { feedInc /= 25; feed[xyz.X] /= 25; feed[xyz.Y] /= 25; feed[xyz.Z] /= 25; homingInfinity /= 25; homingFeed /= 25; }
          else { feedInc *= 25; feed[xyz.X] *= 25; feed[xyz.Y] *= 25; feed[xyz.Z] *= 25; homingInfinity *= 25; homingFeed *= 25; }
        }
        break;
      case 90:
      case 91:
        // set waiting state early to avoid triggering UI event and duplicate command
        WaitingForResponse = true;
        G_AbsoluteMode = (cmd.code == 90)? true : false;
        break;

      case 92:
        for (int i=0; i<xyz.size(); i++)
          if (cmd.has[xyz.toAZ(xyz.X+i)]) position[xyz.X+i] = intCoord(cmd.value[xyz.toAZ(xyz.X+i)]); // value should be 0;
        break;

    }
//    println("position = X"+floatCoord(position[xyz.X])+" Y"+floatCoord(position[xyz.Y])+" Z"+floatCoord(position[xyz.Z]));
  }
  
  // write string to port and consoles
  port.write(s + "\r\n"); println("<= " + s); console_println("< " + s);
  // set waiting state
  WaitingForResponse = true;
  // Todo: intercept tool change commands and enter Paused state
  
}

// initial sequence following port selection to align host state with firmware state
void init_sequence() {
  String[] s = {};
  s = append(s, G_AbsoluteMode? "G90":"G91");
  s = append(s, G_InchMode? "G20":"G21");
  gcode_sequence = s;
  // set up state and send first line
  SendingSequence = true;
  next_gcode_line = 0;
  send_next_line();
}

// homing sequence
void homing_sequence(String axes) {
  String[] seq = {};
  String s1 = "G0";
  String s2 = "G0";
  String s3 = "G1";
  String s4 = "G92";
  int axis_min, axis_max;
  boolean Absolute = G_AbsoluteMode;

  // check if limits are 0
  if (axes == "XY") {
    if (homing_limit[idx.X] == 0 && homing_limit[idx.Y] == 0) { println(": limit <|>0 homes to min|max"); console_println(": limit <|>0 homes to min|max"); return; }
    axis_min = 0; axis_max = 1;
  } else { // if axes == "Z"
    if (homing_limit[idx.Z] == 0) { println(": limit <|>0 homes to min|max"); console_println(": limit <|>0 homes to min|max"); return; }
    axis_min = 2; axis_max = 2;
  }
  
  // Always do homing in relative mode
  if (Absolute) seq = append(seq, "G91");
  
  for (int i=axis_min; i<=axis_max; i++) {    
    // rapid positioning to infinity, rapid stop when limit switches are hit
    if (homing_limit[i] != 0) s1 += " " + idx.strVal[i] + (homing_limit[i]>0 ? homingInfinity : -homingInfinity);    
    // rapid move to future zero
    if (homing_limit[i] != 0) s2 += " " + idx.strVal[i] + (-homing_limit[i]);
    // slow move aiming to overshoot limit switches by x2; stop exactly at limit switches
    if (homing_limit[i] != 0) s3 += " " + idx.strVal[i] + 2*homing_limit[i];
    // set zero if requested
    if (homing_limit[i] != 0) s4 += " " + idx.strVal[i] + "0";
  }
  
  // compose and run sequence
  seq = append(seq, s1);
  seq = append(seq, s2);
  seq = append(seq, s3 + " F" + homingFeed);
  seq = append(seq, s2);
  seq = append(seq, s4);

  // restore absolute mode when done
  if (Absolute) seq = append(seq, "G90");

  gcode_sequence = seq;
  // set up state and send first line
  SendingSequence = true;
  next_gcode_line = 0;
  send_next_line();
}

String jog_string(int jog[], boolean Absolute, boolean RenderZero) {
  EnumXYZ xyz = new EnumXYZ();
  String gcode = RapidPositioning? "G0":"G1";
  float minFeed = 0;
  int totaljog = 0;
  
  for (i=0; i<feed.length; i++) minFeed += feed[i];
  for (i=0; i<jog.length; i++)
    if (jog[i] != 0 || RenderZero) {
      gcode += (" " + xyz.strVal[i] + floatCoord(Absolute ? jog[i] + position[i] : jog[i]));
      totaljog += abs(jog[i]);
      if (feed[i] < minFeed) minFeed = feed[i];
    }
  if (totaljog == 0 && !RenderZero) return "";
  if (!RapidPositioning && (minFeed != lastFeed)) gcode += (" F" + minFeed);
  return gcode;
}

String arc_string(boolean CCW, float R, float start, float end, boolean Absolute) {
  String gcode = CCW? "G3":"G2";
  int start_xy[] = { 0, 0 };
  int end_xy[] = { 0, 0 };

  start_xy = intCoord(polar2cartesian(R, start));
  end_xy = intCoord(polar2cartesian(R, end));
// println("Arc start: "+Arrays.toString(start_xy));
// println("Arc end: "+Arrays.toString(end_xy));

// I,J = -start_xy (I,J are always relative)
// X,Y = end_xy - start_xy (relative)
  gcode += " X" + floatCoord(end_xy[idx.X] - start_xy[idx.X] + (Absolute ? position[idx.X] : 0));
  gcode += " Y" + floatCoord(end_xy[idx.Y] - start_xy[idx.Y] + (Absolute ? position[idx.Y] : 0));
  gcode += " I" + floatCoord(- start_xy[idx.X]);
  gcode += " J" + floatCoord(- start_xy[idx.Y]);

  float minFeed = 0;  
  for (i=0; i<feed.length; i++) minFeed += feed[i];
  for (i=0; i<2; i++) if (feed[i] < minFeed) minFeed = feed[i];
  if (!RapidPositioning && (minFeed != lastFeed)) gcode += (" F" + minFeed);
  
//  println(gcode);
  return gcode;
}

String zero_string() { 
  String s = "ZERO:";
  s += " X" + (G_AbsoluteMode? "0.0" : floatCoord(-position[idx.X]));
  s += " Y" + (G_AbsoluteMode? "0.0" : floatCoord(-position[idx.Y]));
  s += " Z" + (G_AbsoluteMode? "0.0" : floatCoord(-position[idx.Z]));
  return s;
}

String mem_string() {
  String s = "MEM:";
  s += memorySet[idx.X]? (" X"+ floatCoord(G_AbsoluteMode? memory[idx.X] : memory[idx.X]-position[idx.X])) : "";
  s += memorySet[idx.Y]? (" Y"+ floatCoord(G_AbsoluteMode? memory[idx.Y] : memory[idx.Y]-position[idx.Y])) : "";
  s += memorySet[idx.Z]? (" Z"+ floatCoord(G_AbsoluteMode? memory[idx.Z] : memory[idx.Z]-position[idx.Z])) : "";
  return s; 
}

