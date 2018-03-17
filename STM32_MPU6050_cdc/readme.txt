



V02:
- Ported MPU-6050_IMU.ino.. but results do not seem correct..
 
- Setup blackmagic probe debugging in eclipse
  * GDB Hardware Debugging
  * Uncheck "Use remote target"
  * In Startup:-
      Uncheck "Reset and Delay" and "Halt"
      add:-
        target extended-remote \\.\COM12         <<<NOTE: Blackmagic probe enumerates its gdb port as COM12..
        monitor swdp_scan
        attach 1
        mon option erase
        monitor vector_catch disable hard
        set mem inaccessible-by-default off
        set print pretty
  * Uncheck all boxes under "Runtime Options"
  * add to "Run Commands"
      tbreak main
      cont



V01:
- uses blackmagic probe for debug
- cd to output and run debug.bat

