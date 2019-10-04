CXXFLAGS=-Wall -std=c++14 -O3
CC := cc
CXX := g++
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