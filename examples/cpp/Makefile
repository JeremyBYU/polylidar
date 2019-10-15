POLYLIDAR_DIR = ../../polylidar
ROBUST_DIR = ${POLYLIDAR_DIR}/predicates

CXXFLAGS:=$(CXXFLAGS) -Wall -std=c++14 -O3 -g -fopenmp
CC := cc
CXX := g++ # Prefer not to use GCC7, use GCC5,6,8,9; Clang; or MSVC. See WIKI for details.
INCLUDES=-I${POLYLIDAR_DIR} -I${ROBUST_DIR}
LDFLAGS=-L/usr/local/lib # -lprofiler add profiler if you have google-perftools

TARGET=simple

ROBUST_SOURCE_FILES=$(ROBUST_DIR)/constants.c $(ROBUST_DIR)/predicates.c $(ROBUST_DIR)/printing.c $(ROBUST_DIR)/random.c
POLYLIDAR_SOURCE_FILES=$(POLYLIDAR_DIR)/polylidar.cpp $(POLYLIDAR_DIR)/helper.cpp $(POLYLIDAR_DIR)/delaunator.cpp
SOURCE_FILES=${POLYLIDAR_SOURCE_FILES} simple.cpp

OBJFILES_ = $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCE_FILES)))
OBJFILES = $(notdir ${OBJFILES_})
# OBJECTS=polylidar.o helper.o delaunator.o simple.o

all: $(TARGET)

OBJFILESD:$(SOURCE_FILES)
	$(CXX) $(CXXFLAGS) -c $(SOURCE_FILES) $(INCLUDES)

$(TARGET): OBJFILESD
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(OBJFILES) $(INCLUDES) -o ./bin/$(TARGET)
	
clean:
	$(RM) ./bin/simple
	$(RM) $(OBJFILES)