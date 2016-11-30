CXX ?= g++

CXXFLAGS += -c -Wall `pkg-config --cflags opencv`
LDFLAGS += `pkg-config --libs opencv` -Wl,-rpath,/usr/local/lib

LABPREFIX = S

NUM = $(LABPREFIX)$(notdir $(shell pwd))

SRC = $(wildcard *.cpp)
EXE = $(addprefix $(NUM), $(SRC:.cpp=.exe))

all: $(EXE)

$(NUM)%.exe: %.o
	$(CXX) $< -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $< -o $@ $(CXXFLAGS)

clean:
	rm -f $(EXE)





