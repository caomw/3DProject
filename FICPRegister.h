#ifndef _FICP_REGISTER
#define _FICP_REGISTER

#include <string>
#include<iostream>

#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkTransformPolyDataFilter.h>
#include "vtkFractionalIterativeClosestPointTransform.h"

using namespace std;
int FICPRegister(string ref,string target,string savePath);
#endif
