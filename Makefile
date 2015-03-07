EXECUATABLE=ficp
INCLUDE=-I/usr/local/include/vtk-5.10
LIBRARY=-lvtkIO -lvtkFiltering -lvtkCommon \
	-lvtkHybrid -lvtkRendering -lvtkGraphics
CFLAGS=-Wno-deprecated -Wno-unused-parameter

all: $(EXECUATABLE)

$(EXECUATABLE): main.o vtkFractionalIterativeClosestPointTransform.o
		g++ main.o vtkFractionalIterativeClosestPointTransform.o $(LIBRARY) -o $(EXECUATABLE)

main.o: main.cpp
		g++ -c $(INCLUDE) $(CFLAGS) main.cpp 
	
vtkFractionalIterativeClosestPointTransform.o: vtkFractionalIterativeClosestPointTransform.cpp
		g++ -c $(INCLUDE) $(CFLAGS) vtkFractionalIterativeClosestPointTransform.cpp

clean:
		rm $(EXECUATABLE) *.o

