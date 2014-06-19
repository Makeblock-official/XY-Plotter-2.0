@echo off
for %%i in (%0) do set aa=%%~dpi 
cd /d %aa%
cd java/bin
java -Djava.ext.dirs=../../lib -Djava.library.path=../../lib GRemote
echo %aa%