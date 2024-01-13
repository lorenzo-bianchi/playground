# Compiler to use
CXX = g++

# Qhull include and library paths, adjust if necessary
QHULL_INCLUDE = /usr/local/include/
QHULL_LIB = /usr/local/lib/

# Compiler and linker flags
CXXFLAGS = -I$(QHULL_INCLUDE) -Wall -std=c++11
LDFLAGS = -L$(QHULL_LIB) -lqhullcpp -lqhull_r

# Target executable name
TARGET = voronoi

# Source file name
SOURCE = voronoi.cpp

# Rule to build the target
$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

test_voro: test_voro.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

# Rule to clean intermediate files and target
clean:
	rm -f $(TARGET)

# Default rule to be executed when calling 'make' without arguments
all: $(TARGET)