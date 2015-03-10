#ifndef _FICP_REGISTER
#define _FICP_REGISTER

#include <string>
#include<iostream>

#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkDelaunay3D.h>
#include <vtkPolyData.h>
#include <vtkDataObject.h>
#include <vtkUnstructuredGrid.h>
#include <vtkMatrix4x4.h>
#include <vtkGeometryFilter.h>




#include <vtkTransformPolyDataFilter.h>
#include "vtkFractionalIterativeClosestPointTransform.h"

using namespace std;
int FICPRegister(string ref,string target,string savePath);
#endif
