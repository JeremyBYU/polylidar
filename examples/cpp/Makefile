ifdef USE_ROBINHOOD_UNORDERED_MAP
	DEFINE_FLAGS=-DUSE_ROBINHOOD_UNORDERED_MAP=1
else
	DEFINE_FLAGS=
endif
CXXFLAGS=-Wall -std=c++14 -O3 ${DEFINE_FLAGS}
CC := cc
CXX := g++ # DONT USE GCC7, USE GCC5,6,8,9; Clang; or MSVC. See WIKI for details.
INCLUDES=-I../../polylidar
LDFLAGS=-L/usr/local/lib

POLYLIDAR_DIR = ../../polylidar
TARGET=simple


CPPFILES=$(POLYLIDAR_DIR)/polylidar.cpp $(POLYLIDAR_DIR)/helper.cpp $(POLYLIDAR_DIR)/delaunator.cpp simple.cpp
OBJFILES=polylidar.o helper.o delaunator.o simple.o

all: $(TARGET)


OBJFILESD:$(CPPFILES)
	$(CXX) $(CXXFLAGS) -c $(CPPFILES) $(INCLUDES)


$(TARGET): OBJFILESD
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(OBJFILES) $(INCLUDES) -o ./bin/$(TARGET)

# polylidar:$(POLYLIDAR_DIR)/polylidar.cpp $(POLYLIDAR_DIR)/helper.cpp $(POLYLIDAR_DIR)/delaunator.cpp


# all: simple.cpp
	
clean:
	$(RM) ./bin/simple
	$(RM) $(OBJFILES)