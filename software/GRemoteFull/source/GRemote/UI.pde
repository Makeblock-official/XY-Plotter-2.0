// UI and controlP5 related functions
//
// Todo: build update functions for all buttons and toggles, dependent solely on state flags.

Textarea console_ta;
int console_size = 10;
int next_line = 0;
String[] console = new String[console_size];
String[] ord_console = new String[console_size];
String[] jog_buttons = {"X+","X-","Y+","Y-","Z+","Z-"};
String[] jog_toggles = {"arc_mode", "xyz_mode", "zero_mode", "mem_mode"};
String[] jog_frac_name = {"1/32","1/16","1/8","1/4","1/2","1"};
Float[] jog_frac_value = {0.03125, 0.0625, 0.125, 0.25, 0.5, 1.0};
String[] jog_dec_name = {"0.001","0.01","0.1","1","10","20","50","100"};
Float[] jog_dec_value = {0.001, 0.01, 0.1, 1.0, 10.0,20.0,50.0, 100.0};
Integer[] baud_values = {9600, 19200, 38400, 57600, 115200};
DropdownList jogX_ddl, jogY_ddl, jogZ_ddl;
ControlGroup Jogging_grp, Homing_grp, Arcs_grp;
int[] jog_ddl_idx;
boolean[] jog_ddl_frac;

// console functions
void setup_console(int x1, int y1, int x2, int y2) {
  console_ta = cP5.addTextarea("CONSOLE", "", x1,y1,x2,y2);
  for (int i = 0; i < console_size; i++) { console[i] = ""; ord_console[i] = ""; }
  ord_console[0] = "Select serial port";
  Textfield t = cP5.addTextfield("GCODE",x1,y1+110,150,20);
  t.setLabel("");
  cP5.addTextlabel("X_POS","",x1,y1+135);
  cP5.addTextlabel("Y_POS","",x1+50,y1+135);
  cP5.addTextlabel("Z_POS","",x1+100,y1+135);
}

void clear_console() {
  for (int i=0; i<console_size; i++) { console[i]=""; ord_console[i]=""; }
}

void console_println(String s) {
  // add to buffer
  console[next_line] = s;
  if (next_line < console_size-1) next_line++;
  else next_line = 0;
  // reorder console array into ord_console array
  int j = 0; int k = next_line;
  for (int i = k; i < console_size; i++) { ord_console[j] = console[i]; j++; }
  for (int i = 0; i < k; i++) { ord_console[j] = console[i]; j++; }
}

void update_console() {
  console_ta.setText(join(ord_console,'\n'));
  for (int i=0; i<idx.size(); i++) {
    ((Textlabel)cP5.controller(idx.strVal[i]+"_POS")).setVisible(PortResponding);
    ((Textlabel)cP5.controller(idx.strVal[i]+"_POS")).setValue(idx.strVal[i]+floatCoord(position[i]));
  }
  if (port == null || SendingSequence && !Paused) { cP5.controller("GCODE").setVisible(false); } 
  else { 
    cP5.controller("GCODE").setVisible(true);
    if (UI_ClearGCodeTF) {
      ((Textfield)cP5.controller("GCODE")).setText("");
      UI_ClearGCodeTF = false;
    }
    if (XYZMode) ((Textfield)cP5.controller("GCODE")).setText(jog_string(accumulated_jog, G_AbsoluteMode, true));
    if (ZeroMode) ((Textfield)cP5.controller("GCODE")).setText(zero_string());
    if (MemMode) ((Textfield)cP5.controller("GCODE")).setText(mem_string());    
    cP5.controller("GCODE").setLock(XYZMode || ZeroMode || MemMode);
  }
}

// toggle functions
void setup_toggles(int x, int y) {
  Toggle t;
  
  t = cP5.addToggle("sending_LED",false,x,y,14,14);
  t.setLabel("SENDING");
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;
  t.setLock(true);
  t.setColorBackground(color(0,80,0));
  t.setColorActive(color(0,255,0));

  
  t = cP5.addToggle("paused_LED",false,x,y+20,14,14);
  t.setLabel("PAUSED");
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;
  t.setLock(true);
  t.setColorBackground(color(80,80,0));
  t.setColorActive(color(255,255,0));

  t = cP5.addToggle("waiting_LED",false,x,y+40,14,14);
  t.setLabel("WAITING");
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;
  t.setLock(true);
  t.setColorBackground(color(80,0,0));
  t.setColorActive(color(255,0,0));

  t = cP5.addToggle("absolute_mode",false,x+75,y,14,14);
  t.setLabel("ABSOLUTE");
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;
//  t.setLock(true);
  
  t = cP5.addToggle("inch_mode",false,x+75,y+20,14,14);
  t.setLabel("INCHES");
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;
//  t.setLock(true);
}

void update_toggles() {
  // set visibility
  cP5.controller("sending_LED").setVisible(PortResponding);
  cP5.controller("paused_LED").setVisible(PortResponding);
  cP5.controller("waiting_LED").setVisible(PortResponding);
  cP5.controller("absolute_mode").setVisible(PortResponding);
  cP5.controller("inch_mode").setVisible(PortResponding);

  // set lock
  cP5.controller("absolute_mode").setLock(SendingSequence && !Paused);
  cP5.controller("inch_mode").setLock(SendingSequence && !Paused);
  
  // set values
  if ((int)cP5.controller("sending_LED").value() != (SendingSequence? 1:0)) cP5.controller("sending_LED").setValue((SendingSequence? 1:0));
  if ((int)cP5.controller("paused_LED").value() != (Paused? 1:0)) cP5.controller("paused_LED").setValue((Paused? 1:0));
  if ((int)cP5.controller("waiting_LED").value() != (WaitingForResponse? 1:0)) cP5.controller("waiting_LED").setValue((WaitingForResponse? 1:0));
  if ((int)cP5.controller("absolute_mode").value() != (G_AbsoluteMode? 1:0)) cP5.controller("absolute_mode").setValue((G_AbsoluteMode? 1:0));
  if ((int)cP5.controller("inch_mode").value() != (G_InchMode? 1:0)) cP5.controller("inch_mode").setValue((G_InchMode? 1:0));
}

// port selection functions
void setup_port_selector(int x, int y, int x2, int y2) {
  DropdownList baud_ddl = cP5.addDropdownList("BAUD",x,y,45,140);
  DropdownList ports_ddl = cP5.addDropdownList("PORT",x+50,y,80,140);
  baud_ddl.captionLabel().set("115200");
  baud_ddl.captionLabel().style().marginTop = 3;
  baud_ddl.setBarHeight(18);
  for (int i=0; i<baud_values.length; i++) baud_ddl.addItem( baud_values[i].toString(),baud_values[i]);
  
  ports_ddl.captionLabel().set("PORT");
  ports_ddl.captionLabel().style().marginTop = 3;
  ports_ddl.setBarHeight(18);  
  int n_ports = Serial.list().length;
  for(i=0; i<n_ports; i++) ports_ddl.addItem(Serial.list()[i],i);
}

void setup_port_led(int x, int y) {
  Toggle t = cP5.addToggle("port_LED",false,x,y,14,14);
  t.setLabel("");
  t.setLock(true);
  t.setColorBackground(color(0,0,127));
  t.setColorActive(color(0,0,255));
}

void update_port_led() {
  int c = 0;
  if (port != null) c = port.available();
  if (c > 1) c = 1;
  if ((int)cP5.controller("port_LED").value() != c) cP5.controller("port_LED").setValue(c);
}

void update_port_selector() {
  ((DropdownList)cP5.group("PORT")).setVisible(!SendingSequence);
  ((DropdownList)cP5.group("BAUD")).setVisible(!SendingSequence);
}

// buttons

void setup_func_buttons(int x, int y) {
  cP5.addButton("SEND FILE",1,x,y,48,20);  
  cP5.addButton("PAUSE/RESUME",1,x+50,y,48,20);  
  cP5.addButton("CANCEL",1,x+100,y,48,20);    
}

void update_func_buttons() {
  if (!PortResponding) {
    cP5.controller("SEND FILE").setVisible(false);
    cP5.controller("PAUSE/RESUME").setVisible(false);
    cP5.controller("CANCEL").setVisible(false);
    return;
  }
  cP5.controller("SEND FILE").setVisible(!SendingSequence);
  cP5.controller("CANCEL").setVisible(SendingSequence);
  cP5.controller("PAUSE/RESUME").setVisible(SendingSequence);
  if (Paused) cP5.controller("PAUSE/RESUME").setLabel("RESUME");
  else cP5.controller("PAUSE/RESUME").setLabel("PAUSE");
}

void setup_jog_buttons(int x, int y) {  

  Toggle t = cP5.addToggle("arc_mode",false,x,y,48,48);
  t.setLabel("ARCS");
  t.captionLabel().style().marginTop = -31;
  t.captionLabel().style().marginLeft = 14;

  t = cP5.addToggle("xyz_mode",false,x,y+50,48,48);
  t.setLabel("XYZ");
  t.captionLabel().style().marginTop = -31;
  t.captionLabel().style().marginLeft = 14;

  t = cP5.addToggle("zero_mode",false,x+50,y,48,48);
  t.setLabel("ZERO");
  t.captionLabel().style().marginTop = -31;
  t.captionLabel().style().marginLeft = 14;

  t = cP5.addToggle("mem_mode",false,x+50,y+50,48,48);
  t.setLabel("MEM");
  t.captionLabel().style().marginTop = -31;
  t.captionLabel().style().marginLeft = 14;

  cP5.addButton("Z+",1,x+100,y,48,48);    
  cP5.addButton("Z-",1,x+100,y+50,48,48);    

  cP5.addButton("Y+",1,x+50,y+150,48,48);  
  cP5.addButton("Y-",1,x+50,y+200,48,48);  
  cP5.addButton("X-",1,x,y+200,48,48);  
  cP5.addButton("X+",1,x+100,y+200,48,48);
  
  for (int i = 0; i < jog_buttons.length; i++) ((Button)cP5.controller(jog_buttons[i])).activateBy(ControlP5.PRESSED);
}  

void update_jog_buttons() {
  String s;
  Boolean Visible = PortResponding && (!SendingSequence || SendingSequence && Paused);
  
  if ((int)cP5.controller("arc_mode").value() != (ArcMode? 1:0)) cP5.controller("arc_mode").setValue((ArcMode? 1:0));
  if ((int)cP5.controller("xyz_mode").value() != (XYZMode? 1:0)) cP5.controller("xyz_mode").setValue((XYZMode? 1:0));
  if ((int)cP5.controller("zero_mode").value() != (ZeroMode? 1:0)) cP5.controller("zero_mode").setValue((ZeroMode? 1:0));
  if ((int)cP5.controller("mem_mode").value() != (MemMode? 1:0)) cP5.controller("mem_mode").setValue((MemMode? 1:0));
  
  for (int i = 0; i < jog_buttons.length; i++) cP5.controller(jog_buttons[i]).setVisible(Visible);
  for (int i = 0; i < jog_toggles.length; i++) cP5.controller(jog_toggles[i]).setVisible(Visible);
  
  // button labels
  s = "   X+";
  if (ZeroMode) s = "  GO X0";
  if (MemMode) s = "  GO XM";
  if (ArcMode) s = ArcCCW? "  R&UP":"  R&DN";
  cP5.controller("X+").setLabel(s);
  s = "   X-";
  if (ZeroMode) s = " SET X0";
  if (MemMode) s = " SET XM";
  if (ArcMode) s = ArcCCW? "  L&DN":"  L&UP";
  cP5.controller("X-").setLabel(s);

  s = "   Y+";
  if (ZeroMode) s = "  GO Y0";
  if (MemMode) s = "  GO YM";
  if (ArcMode) s = ArcCCW? "  UP&L":"  UP&R";
  cP5.controller("Y+").setLabel(s);
  s = "   Y-";
  if (ZeroMode) s = " SET Y0";
  if (MemMode) s = " SET YM";
  if (ArcMode) s = ArcCCW? "  DN&R":"  DN&L";
  cP5.controller("Y-").setLabel(s);

  s = "   Z+";
  if (ZeroMode) s = "  GO Z0";
  if (MemMode) s = "  GO ZM";
  cP5.controller("Z+").setLabel(s);
  s = "   Z-";
  if (ZeroMode) s = " SET Z0";
  if (MemMode) s = " SET ZM";
  cP5.controller("Z-").setLabel(s);  
}  

void setup_jog_controls(int x, int y, int y_off) { 
  ControlGroup g = cP5.addGroup("GROUP_JOGGING", x, y, 145);
  g.setLabel("JOGGING"); g.close(); Jogging_grp = g;
  
  x = 0;
  y = y_off;

  cP5.addTextlabel("set_jog_label", "SET JOG ", x, y+4).setGroup(g);
  
  Toggle t = cP5.addToggle("fractional_jog", false, x+50, y, 14, 14);
  t.setGroup(g); t.setLabel("FRAC");
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;

  t = cP5.addToggle("rapid_positioning", false, x+95, y, 14, 14);
  t.setGroup(g); t.setLabel("RAPID");
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;
  
  cP5.addTextlabel("jog_z_label", "Z", x, y+54).setGroup(g);
  jogZ_ddl = cP5.addDropdownList("JOG Z", x+15, y+65, 70, y+99);
  jogZ_ddl.setGroup(g);
  jogZ_ddl.captionLabel().set("1");
  jogZ_ddl.captionLabel().style().marginTop = 3;
  jogZ_ddl.setBarHeight(14);

  cP5.addTextlabel("jog_y_label", "Y", x, y+39).setGroup(g);
  jogY_ddl = cP5.addDropdownList("JOG Y", x+15, y+50, 70, y+84);
  jogY_ddl.setGroup(g);
  jogY_ddl.captionLabel().set("1");
  jogY_ddl.captionLabel().style().marginTop = 3;
  jogY_ddl.setBarHeight(14);

  cP5.addTextlabel("jog_x_label", "X", x, y+24).setGroup(g);
  jogX_ddl = cP5.addDropdownList("JOG X", x+15, y+35, 70, y+69);
  jogX_ddl.setGroup(g);
  jogX_ddl.captionLabel().set("1");
  jogX_ddl.captionLabel().style().marginTop = 3;
  jogX_ddl.setBarHeight(14);
  
  Numberbox nbr = cP5.addNumberbox("FEED X", 10, x+95, y+20, 50, 14); nbr.setGroup(g);
  nbr.setLabel("");  nbr.setMin(1); nbr.setMultiplier(1);
  nbr = cP5.addNumberbox("FEED Y", 10, x+95, y+35, 50, 14); nbr.setGroup(g);
  nbr.setLabel("");  nbr.setMin(1); nbr.setMultiplier(1);
  nbr = cP5.addNumberbox("FEED Z", 10, x+95, y+50, 50, 14); nbr.setGroup(g);
  nbr.setLabel("");  nbr.setMin(1); nbr.setMultiplier(1);
  
  int n = FractionalJog ? jog_frac_name.length : jog_dec_name.length;
  for(int i=0; i<n; i++) {
    jogX_ddl.addItem(FractionalJog ? jog_frac_name[i] : jog_dec_name[i], i);
    jogY_ddl.addItem(FractionalJog ? jog_frac_name[i] : jog_dec_name[i], i);
    jogZ_ddl.addItem(FractionalJog ? jog_frac_name[i] : jog_dec_name[i], i);
  }
  jog_ddl_idx = new int[idx.size()];
  jog_ddl_frac = new boolean[idx.size()];
  for (int i=0; i<idx.size(); i++) {
    jog_ddl_idx[i] = i<2?4:3; // index of "1" in jog_dec_value[], default
    jog_ddl_frac[i] = false;
  }
}

void update_jog_controls() {
  Jogging_grp.setVisible(PortResponding && (!SendingSequence || SendingSequence && Paused));
  if ((int)cP5.controller("fractional_jog").value() != (FractionalJog? 1:0)) cP5.controller("fractional_jog").setValue((FractionalJog? 1:0));
  if ((int)cP5.controller("rapid_positioning").value() != (RapidPositioning? 1:0)) cP5.controller("rapid_positioning").setValue((RapidPositioning? 1:0));  
  if (cP5.controller("FEED X").value() != feed[idx.X]) cP5.controller("FEED X").setValue(feed[idx.X]);
  if (cP5.controller("FEED Y").value() != feed[idx.Y]) cP5.controller("FEED Y").setValue(feed[idx.Y]);
  if (cP5.controller("FEED Z").value() != feed[idx.Z]) cP5.controller("FEED Z").setValue(feed[idx.Z]);
  if (UI_ReloadJogDDL) {
    UI_ReloadJogDDL = false;
    jogX_ddl.clear(); jogY_ddl.clear(); jogZ_ddl.clear();
    int n = FractionalJog ? jog_frac_name.length : jog_dec_name.length;
    for(int i=0; i<n; i++) {
      jogX_ddl.addItem(FractionalJog ? jog_frac_name[i] : jog_dec_name[i], i);
      jogY_ddl.addItem(FractionalJog ? jog_frac_name[i] : jog_dec_name[i], i);
      jogZ_ddl.addItem(FractionalJog ? jog_frac_name[i] : jog_dec_name[i], i);
    }
//    println("jog*_ddl reloaded");
//    println("jogX_ddl.value() = "+jogX_ddl.value());
  }
  jogX_ddl.captionLabel().set( jog_ddl_frac[idx.X]? jog_frac_name[jog_ddl_idx[idx.X]] : jog_dec_name[jog_ddl_idx[idx.X]] );
  jogY_ddl.captionLabel().set( jog_ddl_frac[idx.Y]? jog_frac_name[jog_ddl_idx[idx.Y]] : jog_dec_name[jog_ddl_idx[idx.Y]] );
  jogZ_ddl.captionLabel().set( jog_ddl_frac[idx.Z]? jog_frac_name[jog_ddl_idx[idx.Z]] : jog_dec_name[jog_ddl_idx[idx.Z]] );  
  
// ***********************************    
// ***********************************  
// ***********************************  
  
}  

void setup_homing_controls(int x, int y, int y_off) { 
  ControlGroup g = cP5.addGroup("GROUP_HOMING", x, y, 145); 
  g.setLabel("HOMING"); g.close(); Homing_grp = g;

  x = 0;
  y = y_off;
  
  cP5.addTextlabel("homing_limit_label", "LIMITS@ ", 0, y+4).setGroup(g);
  
  Numberbox nbr = cP5.addNumberbox("homing_limit_x", homing_limit[idx.X], x+10, y+20, 35, 14);
  nbr.setLabel("X");  nbr.setMultiplier(0.01); nbr.setGroup(g); nbr.setDecimalPrecision(2);
  nbr.captionLabel().style().marginTop = -14; nbr.captionLabel().style().marginLeft = -10;
  
  nbr = cP5.addNumberbox("homing_limit_y", homing_limit[idx.Y], x+10, y+35, 35, 14);
  nbr.setLabel("Y");  nbr.setMultiplier(0.01); nbr.setGroup(g); nbr.setDecimalPrecision(2);
  nbr.captionLabel().style().marginTop = -14; nbr.captionLabel().style().marginLeft = -10;
  
  nbr = cP5.addNumberbox("homing_limit_z", homing_limit[idx.Z], x+10, y+50, 35, 14);
  nbr.setLabel("Z");  nbr.setMultiplier(0.01); nbr.setGroup(g); nbr.setDecimalPrecision(2);
  nbr.captionLabel().style().marginTop = -14; nbr.captionLabel().style().marginLeft = -10;

  nbr = cP5.addNumberbox("homing_infinity", homingInfinity, x+55, y+20, 45, 14);
  nbr.setLabel("INFINITY");  nbr.setMultiplier(1); nbr.setMin(1); nbr.setGroup(g);
  nbr.captionLabel().style().marginTop = 0; nbr.captionLabel().style().marginLeft = 0;

  nbr = cP5.addNumberbox("homing_feed", homingFeed, x+105, y+20, 40, 14);
  nbr.setLabel("FEED");  nbr.setMultiplier(1); nbr.setMin(1); nbr.setGroup(g);
  nbr.captionLabel().style().marginTop = 0; nbr.captionLabel().style().marginLeft = 0;

  Toggle t = cP5.addToggle("homing_set_zero", true, x+55, y, 14, 14);
  t.setLabel("SET ZERO"); t.setGroup(g);
  t.captionLabel().style().marginTop = -14;
  t.captionLabel().style().marginLeft = 18;

  cP5.addButton("HOME XY", 1, x+55, y+50, 45, 14).setGroup(g);
  cP5.addButton("HOME Z", 1, x+105, y+50, 40, 14).setGroup(g);
}

void update_homing_controls() { 
  Homing_grp.setVisible(PortResponding && (!SendingSequence || SendingSequence && Paused));
  if ((int)cP5.controller("homing_set_zero").value() != (HomingSetZero? 1:0)) cP5.controller("homing_set_zero").setValue((HomingSetZero? 1:0));
  if (cP5.controller("homing_infinity").value() != homingInfinity) cP5.controller("homing_infinity").setValue(homingInfinity);
  if (cP5.controller("homing_feed").value() != homingFeed) cP5.controller("homing_feed").setValue(homingFeed);  
}

void setup_arc_controls(int x, int y, int y_off) {
  ControlGroup g = cP5.addGroup("GROUP_ARCS", x, y, 145);
  g.setLabel("ARCS"); g.close(); Arcs_grp = g;

  x = 0;
  y = y_off;

  Textfield tf = cP5.addTextfield("ARC_RADIUS", x+10, y, 50, 20); tf.setGroup(g); tf.setLabel("R");
  tf.captionLabel().style().marginTop = -17;
  tf.captionLabel().style().marginLeft = -10;
//  cP5.addTextlabel("radius_label","R",x,y+7).setGroup(g);

  Toggle t = cP5.addToggle("ARC_CCW",false,x+115,y,30,20);
  t.setGroup(g); t.setLabel("CCW CW"); t.setMode(ControlP5.SWITCH);
  t.captionLabel().style().marginTop = -17;
  t.captionLabel().style().marginLeft = -42;

  tf = cP5.addTextfield("ARC_START", x+33, y+26, 40, 20); tf.setGroup(g); tf.setLabel("START@");
  tf.captionLabel().style().marginTop = -17;
  tf.captionLabel().style().marginLeft = -33;

  tf = cP5.addTextfield("ARC_END", x+105, y+26, 40, 20); tf.setGroup(g); tf.setLabel("END@");
  tf.captionLabel().style().marginTop = -17;
  tf.captionLabel().style().marginLeft = -25;

  ((Textfield)cP5.controller("ARC_RADIUS")).setText(String.valueOf(arc_radius));
  ((Textfield)cP5.controller("ARC_START")).setText(String.valueOf(arc_start));
  ((Textfield)cP5.controller("ARC_END")).setText(String.valueOf(arc_end));
}

void update_arc_controls() {
  Arcs_grp.setVisible(PortResponding && (!SendingSequence || SendingSequence && Paused));
  if (UI_ReloadArcTF) {
    ((Textfield)cP5.controller("ARC_RADIUS")).setText(String.valueOf(arc_radius));
    ((Textfield)cP5.controller("ARC_START")).setText(String.valueOf(arc_start));
    ((Textfield)cP5.controller("ARC_END")).setText(String.valueOf(arc_end));
    UI_ReloadArcTF = false;
  }
  if ((int)cP5.controller("ARC_CCW").value() != (ArcCCW? 1:0)) cP5.controller("ARC_CCW").setValue((ArcCCW? 1:0));
}

void update_textfields() {
  if (UI_ClearFocusTF) {
    ((Textfield)cP5.controller("GCODE")).setFocus(false);
    ((Textfield)cP5.controller("ARC_RADIUS")).setFocus(false);
    ((Textfield)cP5.controller("ARC_START")).setFocus(false);
    ((Textfield)cP5.controller("ARC_END")).setFocus(false);
    UI_ClearFocusTF = false;
  }
}

void open_group(char g) {
  if (g == 'J') {Jogging_grp.open();Homing_grp.close();Arcs_grp.close();} else Jogging_grp.close();
  if (g == 'H') {Homing_grp.open();Arcs_grp.close();Jogging_grp.close();} else Homing_grp.close();
  if (g == 'A') {Arcs_grp.open();Jogging_grp.close();Homing_grp.close();} else Arcs_grp.close();
}

void update_groups() {
  Jogging_grp.setColorLabel(Jogging_grp.isOpen() ? 0xFFFFFFFF : 0xFF888888); // 0xFF08A2CF);
  Homing_grp.setColorLabel(Homing_grp.isOpen() ? 0xFFFFFFFF : 0xFF888888);
  Arcs_grp.setColorLabel(Arcs_grp.isOpen() ? 0xFFFFFFFF : 0xFF888888);  
}

