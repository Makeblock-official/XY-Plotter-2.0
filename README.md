[![alt text](images/Logo.png "Makeblock Logo") Library v2.1.0422](https://www.Makeblock.cc)



How to use:

You will get a complete instruction of how to construct the XY Plotter 2.0 in assembly instruction.pdf. As for the software, you could refer to the software manaual.pdf. You could learn how to run the XY plotter 2.0 in the basic version, and the complete version will tell you more details.

Here is a brief instruction about the software:

1. Download and install Processing-2.1.2，http://processing.org/ 

2. Download the software package and decompress it，https://github.com/Makeblock-official/XY-Plotter-2.0/archive/master.zip

3. Connect Me baseboard to computer with micro USB cable.

4. Open GCodepraser->GCodeParser.ino by Arduino IDE. Click Tools->Serial Ports, choose COM XX (Not COM1 and COM2). Click Tools->Boards, choose Arduino Leonardo. At last, click "upload" button on the right-top corner.
[![alt text](images/Upload.png "Upload program to Me Baseboard")](https://www.Makeblock.cc)

5. Close Arduino IDE, open GRemote->GRemote.pde. Click "run" button on the right-top corner.

6. Now, you could control the XY-Plotter 2.0 by mouse and keyboard. And, you can also run it with Gcode. You could generate Gcode file by another software (dxf2gcode(https://code.google.com/p/dxf2gcode/) or any other capable). You should save the Gcode file with .cnc and open it by GRemote.

[![alt text](images/XY Plotter 2.0.jpg "Makeblock XY Plotter 2.0")](http://www.makeblock.cc/xy-plotter-robot-kit-2-0/)
