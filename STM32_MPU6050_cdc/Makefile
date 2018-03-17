#
# For debug build, run make DEBUG_BUILD=1 from commandline.
# For Release build, just run make.
#
# If get assertion failure from linking: assertion fail in file bfd/elf32-arm.c
# then ensure use the latest gcc.. Tried gcc-arm-none-eabi-4_9-2014q4-20141203-win32.zip from launchpad is ok, but
# gcc-arm-none-eabi-4_8-2014q1-20140314-win32.zip will give the failure.. also earlier 4.7 version from emIDE fails also.
# This failure is caused by the options: -ffunction-sections -fdata-sections -Wl,--gc-sections
#
#

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME = STM32_MPU6050

DEBUG_BUILD := 1
# uncomment for softfloat..
SOFTFLOAT = 1

# Put your stlink folder here so make burn will work.
STLINK=~/stlink.git

OBJDIR=obj
OUTDIR=output

DEPDIR=dep

# Sources
#=====================
USER = Src
USER_SRC = $(USER)/main.c $(USER)/stm32f10x_it.c $(USER)/SEGGER_HardFaultHandler.c \
		$(USER)/MPU6050.c $(USER)/delay.c $(USER)/st7735.c \
		$(USER)/parser.c \
		$(USER)/ringbuffer.c $(USER)/usart.c

USB_CDC_SRC = $(wildcard $(USER)/usb/*.c)

USER_SRC += $(USB_CDC_SRC)

ifneq ($(DEBUG_BUILD),1)
# need syscalls.c for release build
USER_SRC +=	$(USER)/syscalls.c
endif

CMSIS = Src/Libraries/CMSIS/CM3
CMSIS_SRC = $(CMSIS)/DeviceSupport/ST/STM32F10x/system_stm32f10x.c

MIDDLEWARES = Src/Libraries/STM32_USB-FS-Device_Driver
MIDDLEWARES_SRC = $(wildcard $(MIDDLEWARES)/src/*.c) 

HAL = Src/Libraries/STM32F10x_StdPeriph_Driver
HAL_SRC = $(wildcard $(HAL)/src/*.c)

SRCS = $(USER_SRC) $(MIDDLEWARES_SRC) $(CMSIS_SRC) $(HAL_SRC)

# add startup file to build
ASM1_SRCS = Setup/startup_stm32f10x_md.s
ASM2_SRCS = $(USER)/HardFaultHandler.S

# Include path
#=====================
INCLUDES  = -I.

# Include files from STM libraries
INCLUDES += -I$(USER) -I$(USER)/usb
INCLUDES += -I$(CMSIS)/CoreSupport
INCLUDES += -I$(CMSIS)/DeviceSupport/ST/STM32F10x
INCLUDES += -I$(MIDDLEWARES)/Inc
INCLUDES += -I$(HAL)/Inc


# Objects
#=====================
OBJS = $(patsubst %.o, $(OBJDIR)/%.o,$(SRCS:.c=.o))
ASM1_OBJECTS = $(patsubst %.o, $(OBJDIR)/%.o,$(ASM1_SRCS:.s=.o))
ASM2_OBJECTS = $(patsubst %.o, $(OBJDIR)/%.o,$(ASM2_SRCS:.S=.o))

ASM_OBJECTS = $(ASM1_OBJECTS) $(ASM2_OBJECTS)

ALLOBJS = $(ASM_OBJECTS)  $(OBJS)
#DEPS := $(ALLOBJS:.o=.d)


#######################################################################################
#GCC_PATH = "c:/work/utils/launchpad-gcc/bin"
#GCC_PATH = "c:/work/UTILS/emIDE V2.20/arm/bin"
GCC_PATH = "d:/utility/launchpad/bin"

CC			= $(GCC_PATH)/arm-none-eabi-gcc
LD			= $(GCC_PATH)/arm-none-eabi-gcc
OBJCOPY		= $(GCC_PATH)/arm-none-eabi-objcopy
SIZE        = $(GCC_PATH)/arm-none-eabi-size
CP          = cp -p
RM          = rm -rf
MV          = mv
MKDIR       = -mkdir -p 2> /dev/null

# Compiler flags
#=====================
CFLAGS += -g -Wall 
CFLAGS += -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -mthumb-interwork
CFLAGS += -std=c99
CFLAGS += -funsigned-char -Wl,-lnosys

CFLAGS += -TSetup/stm32_flash.ld -Xlinker -Map=$(OUTDIR)/$(PROJ_NAME).map

CFLAGS_RM_UNUSED_SECT = -ffunction-sections -fdata-sections -Wl,--gc-sections 

CFLAGS_SPECS = --specs=rdimon.specs
# --specs=nano.specs  - use this to further reduce code size

#Tell linker to garbage collect and discard unused sections
#Strip the debug information, so as to make the code as small as possible.  
CFLAGS_RELEASE = $(CFLAGS_RM_UNUSED_SECT)
#CFLAGS_RELEASE += -O2 -Wl,-s

#Debug version
CFLAGS_DEBUG = -DDEBUG -g 
CFLAGS_DEBUG += $(CFLAGS_RM_UNUSED_SECT) 
CFLAGS_DEBUG += -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group

CFLAGS += $(CFLAGS_SPECS)


# -mfloat-abi=name
# Specifies which floating-point ABI to use. Permissible values are: `soft', `softfp' and `hard'.
# Specifying `soft' causes GCC to generate output containing library calls for floating-point operations. (ie, software emulation of floating point operations)
# `softfp' allows the generation of code using hardware floating-point instructions, but still uses the soft-float calling conventions. 
# `hard' allows generation of floating-point instructions and uses FPU-specific calling conventions.
#
# The default depends on the specific target configuration. Note that the hard-float and soft-float ABIs are not link-compatible; 
# you must compile your entire program with the same ABI, and link with a compatible set of libraries. 
#
# -mhard-float
# Equivalent to -mfloat-abi=hard. 
# -msoft-float
# Equivalent to -mfloat-abi=soft. 
#
ifndef SOFTFLOAT
 #For hard float (uses fpu, passes the parameters in FPU registers)
 CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
 CFLAGS += -fsingle-precision-constant -Wdouble-promotion
else
 #For soft float (uses floating point library, parameters are passed in core registers)
 CFLAGS += -msoft-float
endif

ifeq ($(DEBUG_BUILD),1)
CFLAGS += $(CFLAGS_DEBUG)
else
CFLAGS += $(CFLAGS_RELEASE)
endif


# Generate dependency information
CFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d

# Targets
#=====================

all: $(OBJDIR) proj size
.PHONY: all proj size

proj: $(OUTDIR)/$(PROJ_NAME).elf

# final output
$(OUTDIR)/$(PROJ_NAME).elf: $(ALLOBJS)
	@echo
	@echo BUILDING ELF $@
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(OUTDIR)/$(PROJ_NAME).elf $(OUTDIR)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(OUTDIR)/$(PROJ_NAME).elf $(OUTDIR)/$(PROJ_NAME).bin

$(OBJDIR):
	@$(MKDIR) $(OBJDIR)
	@$(MKDIR) $(OUTDIR)
	@$(MKDIR) $(DEPDIR)

# rule to build from c source
$(OBJDIR)/%.o: %.c
	@$(MKDIR) $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) $< -c -o $@

# rule to build from assembly source
$(OBJDIR)/%.o: %.S %.s
	@$(MKDIR) $(dir $@)
	$(CC) $(CFLAGS) $< -c -o $@


# Calculate the Size of the Image
#--------------------------------
size: $(OUTDIR)/$(PROJ_NAME).elf
	@echo
	@$(SIZE) $<


clean:
	$(RM) $(OBJDIR)
	$(RM) $(OUTDIR)
	$(RM) $(DEPDIR)
	$(RM) *~


# Flash the STM32
burn: all
	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x8000000


# debugging, use make print-MACRONAME, e.g. make print-SRCS
print-%  : ; @echo $* = $($*)	
	
	
# Include the dependency files.
-include $(wildcard $(DEPDIR)/*)
