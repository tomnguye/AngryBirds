CXX = g++
CXXFLAGS = -g -Wall -std=c++17 -I/usr/include/eigen3 -I/usr/include/opencv4
LDFLAGS  = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs
VPATH = ..

SRCS = $(wildcard ../*.cpp)
OBJS = $(notdir $(SRCS:.cpp=.o))

main: $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(OBJS) main