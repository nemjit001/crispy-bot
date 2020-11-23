CC = arm-none-eabi-g++
CFLAGS = -Wall -Werror
LDLIBS = 
TARGET = crispy_bot
OBJCOPY = arm-none-eabi-objcopy

SRCDIR=src
OBJDIR=obj

SRCS=$(wildcard $(SRCDIR)/*.cpp)
HDRS=$(wildcard $(SRCDIR)/*.cpp)
_OBJ=$(patsubst $(SRCDIR)/%, $(OBJDIR)/%, $(SRCS:.cpp=.o))
OBJ=$(filter-out $(SRCDIR)/$(TARGET).o, $(_OBJ))

.phony: all clean flash

all: $(TARGET)

flash:

$(TARGET): $(OBJ) $(OBJDIR)/$(TARGET).o
	$(CC) --specs=nosys.specs -o $@ $^ $(CFLAGS) $(LDLIBS)
	$(OBJCOPY) -O binary $(TARGET) $(TARGET).bin
	$(OBJCOPY) -O ihex $(TARGET) $(TARGET).hex

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	+@[ -d $(OBJDIR) ] || mkdir -p $(OBJDIR)
	$(CC) --specs=nosys.specs -MMD $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJDIR)/* $(TARGET) $(TARGET).hex $(TARGET).bin
