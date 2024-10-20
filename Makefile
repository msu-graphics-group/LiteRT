CC  = gcc
CXX = g++
LD  = g++

SRC = src/
OBJ = obj/
INC = inc/

INCFLAGS  = -I$(INC)

CXXFLAGS  = $(INCFLAGS)
CCFLAGS   = $(INCFLAGS)

CXXFLAGS += -std=c++11
CXXFLAGS += -Xpreprocessor
CXXFLAGS += -Wno-deprecated-declarations
CXXFLAGS += -O2

CCFLAGS  += -std=c17
CCFLAGS  += -O2

LDFLAGS += -O2

SRCFILES  = $(SRC)main.cpp
OBJFILES  = $(OBJ)main.o
TARGET    = main

### Modules ###
MATHMOD   = $(MOD)LiteMath/
ALLMODS   = $(MATHMOD)

# Target binary
$(TARGET): $(OBJFILES)
	$(LD) $(LDFLAGS) -o $@ $^

$(OBJ)%.o: $(SRC)%.cpp $(INC)%.h
	$(CXX) -c $(CXXFLAGS) -o $@ $<

test:
	echo $(OBJFILES)

.PHONY: run
run: $(TARGET)
	./$(TARGET)

.PHONY: clean
clean:
	rm -f $(OBJFILES)
