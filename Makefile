SRCDIR=src
OBJDIR=obj
LIBDIR=lib

CC = arm-none-eabi-g++
CFLAGS = -Wall -Werror -Wextra -Wpedantic
LDLIBS = -lpixy2
TARGET = crispy_bot
OBJCOPY = arm-none-eabi-objcopy

SRCS=$(wildcard $(SRCDIR)/*.cpp)
HDRS=$(wildcard $(SRCDIR)/*.h)
_OBJ=$(patsubst $(SRCDIR)/%, $(OBJDIR)/%, $(SRCS:.cpp=.o))
OBJ=$(filter-out $(OBJDIR)/$(TARGET).o, $(_OBJ))

.phony: all clean flash

all: $(TARGET)

flash: all
	@echo flashing to board

$(TARGET): $(TARGET).elf

$(TARGET).elf: $(OBJ) $(OBJDIR)/$(TARGET).o
	$(CC) --specs=nosys.specs $(CFLAGS) -L$(LIBDIR) $(LDLIBS) -o $@ $^
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin
	$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	+@[ -d $(OBJDIR) ] || mkdir -p $(OBJDIR)
	$(CC) --specs=nosys.specs -MMD $(CFLAGS) -L$(LIBDIR) $(LDLIBS) -c -o $@ $<

clean:
	rm -f $(OBJDIR)/* $(TARGET).elf $(TARGET).hex $(TARGET).bin
