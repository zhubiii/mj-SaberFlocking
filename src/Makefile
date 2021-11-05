COMMON=-O2 -I../include -L../bin -std=c++11 -mavx -pthread -Wl,-rpath,'$$ORIGIN'

all:
	g++ $(COMMON) mybasic.cc      -lmujoco210 -framework OpenGL -lglew ../bin/libglfw.3.dylib -o ../bin/mybasic
