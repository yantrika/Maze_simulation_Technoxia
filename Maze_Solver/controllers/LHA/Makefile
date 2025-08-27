CXX = g++
CXXFLAGS = -O2 -Wall -Wextra -std=c++17
WEBOTS_HOME_PATH = $(WEBOTS_HOME)
INCLUDE = -I"$(WEBOTS_HOME_PATH)/include/controller/cpp"
LIBS = -L"$(WEBOTS_HOME_PATH)/lib/controller" -lCppController -lm

TARGET = LHA
SOURCES = main.cpp RobotController.cpp
OBJECTS = $(SOURCES:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
