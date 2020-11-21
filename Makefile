CC=g++
CFLAGS=-Wall -Wpedantic
LDLIBS=

TARGET=bot

SRCDIR=src
ODIR=obj

SRCS=$(wildcard $(SRCDIR)/*.cpp)
HDRS=$(wildcard $(SRCDIR)/*.h)
_OBJ=$(patsubst $(SRCDIR)/%, $(ODIR)/%, $(SRCS:.cpp=.o))
OBJ=$(filter-out $(ODIR)/$(TARGET).o, $(_OBJ))

.phony: all clean

all: $(TARGET)

$(TARGET): $(OBJ) $(ODIR)/$(TARGET).o
	@echo $(SRCS)
	$(CC) -o $@ $^ $(CFLAGS) $(LDLIBS)

$(ODIR)/%.o: $(SRCDIR)/%.cpp
	+@[ -d $(ODIR) ] || mkdir -p $(ODIR)
	$(CC) -MMD $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(ODIR)/* $(TARGET)
