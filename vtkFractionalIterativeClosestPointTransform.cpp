/*=========================================================================

Program:   Visualization Toolkit
Module:    $RCSfile: vtkFractionalIterativeClosestPointTransform.cxx,v $

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkFractionalIterativeClosestPointTransform.h"
//#include "timing.h"

#include <vtkCellLocator.h>
#include <vtkDataSet.h>
#include <vtkLandmarkTransform.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkTransform.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkLookupTable.h>
#include <vtkPolyDataMapper.h>
#include <vtkLightKit.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPLYWriter.h>
#include <vtkSmartPointer.h>

//#include <Windows.h>

vtkCxxRevisionMacro(vtkFractionalIterativeClosestPointTransform, "$Revision: 1.14 $");
vtkStandardNewMacro(vtkFractionalIterativeClosestPointTransform);

double GetTickCount()
{
    double t;
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    t = ts.tv_sec;
    t = t * 1000.0 + ts.tv_nsec / 1000000.0;

    return t;
}

//----------------------------------------------------------------------------
int compare(const void * a,const void * b)
{
	if (((DISTELEM*)a)->dist==((DISTELEM*)b)->dist)
		return 0;
	else
		if (((DISTELEM*)a)->dist < ((DISTELEM*)b)->dist)
			return -1;
		else
			return 1;
}

vtkFractionalIterativeClosestPointTransform::vtkFractionalIterativeClosestPointTransform()
: vtkLinearTransform()
{
	this->Source = NULL;
	this->Target = NULL;
	this->Locator = NULL;
	this->LandmarkTransform = vtkLandmarkTransform::New();
	this->MaximumNumberOfIterations = 50;
	this->CheckMeanDistance = 0;
	this->MeanDistanceMode = VTK_ICP_MODE_RMS;
	this->MaximumMeanDistance = 0.01;
	this->MaximumNumberOfLandmarks = 200;
	this->StartByMatchingCentroids = 0;
	this->VisualizeOn = 1;

	this->NumberOfIterations = 0;
	this->MeanDistance = 0.0;

	// Visualization
	// this->VrenWin = vtkRenderWindow::New();
	// this->Ren = vtkRenderer::New();
	// this->VrenWin->AddRenderer(this->Ren);
	// this->VrenWin->SetSize(800,800);
	this->VrenWin = NULL;
	this->Ren = NULL;
	this->plyMapT= vtkPolyDataMapper::New();
	this->plyMapS= vtkPolyDataMapper::New();
	this->plyMapSP= vtkPolyDataMapper::New();
	this->cactorT = vtkActor::New();
	this->cactorS = vtkActor::New();
	this->cactorSP = vtkActor::New();
	this->lk = vtkLightKit::New();
	this->iCPTransFilter = vtkTransformPolyDataFilter::New();
	this->iren = vtkRenderWindowInteractor::New();
}

//----------------------------------------------------------------------------

const char *vtkFractionalIterativeClosestPointTransform::GetMeanDistanceModeAsString()
{
	if ( this->MeanDistanceMode == VTK_ICP_MODE_RMS )
	{
		return "RMS";
	}
	else
	{
		return "AbsoluteValue";
	}
}

//----------------------------------------------------------------------------

vtkFractionalIterativeClosestPointTransform::~vtkFractionalIterativeClosestPointTransform()
{
	ReleaseSource();
	ReleaseTarget();
	ReleaseLocator();
	this->LandmarkTransform->Delete();
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::SetSource(vtkDataSet *source)
{
	if (this->Source == source)
	{
		return;
	}

	if (this->Source)
	{
		this->ReleaseSource();
	}

	if (source)
	{
		source->Register(this);
	}

	this->Source = source;
	this->Modified();
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::ReleaseSource(void) {
	if (this->Source) 
	{
		this->Source->UnRegister(this);
		this->Source = NULL;
	}
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::SetTarget(vtkDataSet *target)
{
	if (this->Target == target)
	{
		return;
	}

	if (this->Target)
	{
		this->ReleaseTarget();
	}

	if (target)
	{
		target->Register(this);
	}

	this->Target = target;
	this->Modified();
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::ReleaseTarget(void) {
	if (this->Target) 
	{
		this->Target->UnRegister(this);
		this->Target = NULL;
	}
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::SetLocator(vtkCellLocator *locator)
{
	if (this->Locator == locator)
	{
		return;
	}

	if (this->Locator)
	{
		this->ReleaseLocator();
	}

	if (locator)
	{
		locator->Register(this);
	}

	this->Locator = locator;
	this->Modified();
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::ReleaseLocator(void) {
	if (this->Locator) 
	{
		this->Locator->UnRegister(this);
		this->Locator = NULL;
	}
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::CreateDefaultLocator() {
	if (this->Locator) 
	{
		this->ReleaseLocator();
	}

	this->Locator = vtkCellLocator::New();
}

//------------------------------------------------------------------------

unsigned long vtkFractionalIterativeClosestPointTransform::GetMTime()
{
	unsigned long result = this->vtkLinearTransform::GetMTime();
	unsigned long mtime;

	if (this->Source)
	{
		mtime = this->Source->GetMTime(); 
		if (mtime > result)
		{
			result = mtime;
		}
	}

	if (this->Target)
	{
		mtime = this->Target->GetMTime(); 
		if (mtime > result)
		{
			result = mtime;
		}
	}

	if (this->Locator)
	{
		mtime = this->Locator->GetMTime(); 
		if (mtime > result)
		{
			result = mtime;
		}
	}

	if (this->LandmarkTransform)
	{
		mtime = this->LandmarkTransform->GetMTime();
		if (mtime > result)
		{
			result = mtime;
		}
	}

	return result;
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::Inverse()
{
	vtkDataSet *tmp1 = this->Source;
	this->Source = this->Target;
	this->Target = tmp1;
	this->Modified();
}

//----------------------------------------------------------------------------

vtkAbstractTransform *vtkFractionalIterativeClosestPointTransform::MakeTransform()
{
	return vtkFractionalIterativeClosestPointTransform::New(); 
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::InternalDeepCopy(vtkAbstractTransform *transform)
{
	vtkFractionalIterativeClosestPointTransform *t = (vtkFractionalIterativeClosestPointTransform *)transform;

	this->SetSource(t->GetSource());
	this->SetTarget(t->GetTarget());
	this->SetLocator(t->GetLocator());
	this->SetMaximumNumberOfIterations(t->GetMaximumNumberOfIterations());
	this->SetCheckMeanDistance(t->GetCheckMeanDistance());
	this->SetMeanDistanceMode(t->GetMeanDistanceMode());
	this->SetMaximumMeanDistance(t->GetMaximumMeanDistance());
	this->SetMaximumNumberOfLandmarks(t->GetMaximumNumberOfLandmarks());

	this->Modified();
}

//----------------------------------------------------------------------------
void render(vtkTransform * accumulate, vtkDataSet * poly, vtkRenderWindow * renWin)
{
	vtkSmartPointer<vtkTransformPolyDataFilter> iCPTransFilter =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	iCPTransFilter->SetInput((vtkPolyData *)poly);
	iCPTransFilter->SetTransform(accumulate);
	iCPTransFilter->Update();

	vtkRenderer *Ren1 = vtkRenderer::New();

	renWin->AddRenderer(Ren1);
	renWin->SetSize(300,300);

	vtkLookupTable *lut = vtkLookupTable::New();
	lut->SetNumberOfColors(256);
	lut->SetHueRange(1.0,1.0);
	lut->SetSaturationRange(0,0);
	lut->SetValueRange(1.0,1.0);
	lut->SetAlphaRange(1.0,1.0);
	lut->SetRange(0,4);

	vtkPolyDataMapper * plyMap =vtkPolyDataMapper::New();
	plyMap->SetInput(iCPTransFilter->GetOutput());

	vtkActor *cactor = vtkActor::New();
	cactor->SetMapper(plyMap);

	Ren1->AddActor(cactor);

	vtkLightKit *lk = vtkLightKit::New();
	lk->SetKeyLightIntensity(0.7);
	lk->AddLightsToRenderer(Ren1);

	/*vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);
	iren->Initialize();
	iren->Start();*/
	renWin->Render();

	Ren1->Delete();
	lut->Delete();
	plyMap->Delete();
	cactor->Delete();
	lk->Delete();
}

//----------------------------------------------------------------------------
void Initialize(vtkTransform * accumulate, vtkDataSet * poly,vtkRenderWindow * renWin)
{
	vtkSmartPointer<vtkTransformPolyDataFilter> iCPTransFilter =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	iCPTransFilter->SetInput((vtkPolyData *)poly);
	iCPTransFilter->SetTransform(accumulate);
	iCPTransFilter->Update();

	vtkRenderer *Ren1 = vtkRenderer::New();

	renWin->AddRenderer(Ren1);
	renWin->SetSize(300,300);

	vtkLookupTable *lut = vtkLookupTable::New();
	lut->SetNumberOfColors(256);
	lut->SetHueRange(1.0,1.0);
	lut->SetSaturationRange(0,0);
	lut->SetValueRange(1.0,1.0);
	lut->SetAlphaRange(1.0,1.0);
	lut->SetRange(0,4);

	vtkPolyDataMapper * plyMap =vtkPolyDataMapper::New();
	plyMap->SetInput(iCPTransFilter->GetOutput());

	vtkActor *cactor = vtkActor::New();
	cactor->SetMapper(plyMap);

	Ren1->AddActor(cactor);

	vtkLightKit *lk = vtkLightKit::New();
	lk->SetKeyLightIntensity(0.7);
	lk->AddLightsToRenderer(Ren1);

	/*vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);
	iren->Initialize();
	iren->Start();*/

	renWin->Render();

	Ren1->Delete();
	lut->Delete();
	plyMap->Delete();
	cactor->Delete();
	lk->Delete();
}

//----------------------------------------------------------------------------

void renderI(vtkDataSet * poly,vtkRenderWindow * renWin)
{
	vtkRenderer *Ren1 = vtkRenderer::New();

	renWin->AddRenderer(Ren1);
	renWin->SetSize(300,300);

	vtkLookupTable *lut = vtkLookupTable::New();
	lut->SetNumberOfColors(256);
	lut->SetHueRange(1.0,1.0);
	lut->SetSaturationRange(0,0);
	lut->SetValueRange(1.0,1.0);
	lut->SetAlphaRange(1.0,1.0);
	lut->SetRange(0,4);

	vtkPolyDataMapper * plyMap =vtkPolyDataMapper::New();
	plyMap->SetInput((vtkPolyData *)poly);

	vtkActor *cactor = vtkActor::New();
	cactor->SetMapper(plyMap);

	Ren1->AddActor(cactor);

	vtkLightKit *lk = vtkLightKit::New();
	lk->SetKeyLightIntensity(0.7);
	lk->AddLightsToRenderer(Ren1);

	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);
	iren->Initialize();
	iren->Start();
}

//----------------------------------------------------------------------------
void vtkFractionalIterativeClosestPointTransform::Visualize(vtkTransform * accumulate,int tag)
{
	// Three modes 
	// tag == 0 normal visualization
	// tag == 1 display result when ends
	// tag == 2 let the users initialize the Sources
	if (this->Ren == NULL)
	{
		this->VrenWin = vtkRenderWindow::New();
		this->Ren = vtkRenderer::New();
		this->VrenWin->AddRenderer(this->Ren);
		this->VrenWin->SetSize(800,800);
	}
	int Vstep =1;

	if (this->NumberOfIterations%Vstep != 0 && tag == 0)
		return;
	
	this->iCPTransFilter->SetInput((vtkPolyData *)this->GetSource());
	this->iCPTransFilter->SetTransform(accumulate);
	this->iCPTransFilter->Update();

	this->plyMapT->SetInput((vtkPolyData *)this->GetTarget());
	this->plyMapS->SetInput(this->iCPTransFilter->GetOutput());	
	this->plyMapSP->SetInput(this->iCPTransFilter->GetOutput());
	this->plyMapS->ScalarVisibilityOff();
	this->plyMapSP->ScalarVisibilityOn();

	this->cactorT->SetMapper(plyMapT);
	this->cactorT->GetProperty()->SetRepresentationToWireframe();
	this->cactorT->GetProperty()->SetColor(0.0,1.0,0.0);
	this->cactorT->GetProperty()->SetOpacity(0.05);
	this->cactorS->SetMapper(plyMapS);	
	this->cactorS->GetProperty()->SetRepresentationToSurface();
	this->cactorS->GetProperty()->SetOpacity(1);
	this->cactorSP->SetMapper(plyMapSP);
	this->cactorSP->GetProperty()->SetRepresentationToWireframe();
	this->cactorSP->GetProperty()->SetOpacity(1);
	this->cactorSP->GetProperty()->SetPointSize(2);

	this->Ren->AddActor(cactorT);
	//this->Ren->AddActor(cactorS);
	this->Ren->AddActor(cactorSP);

	// this->lk->SetKeyLightIntensity(0.4);
	// this->lk->AddLightsToRenderer(Ren);

	if (tag == 0)
	{		
		this->Ren->GetRenderWindow()->Render();
		// this->VrenWin->Render();
		return;
	}
	/*if (tag == 1)
	{
		this->Ren->GetRenderWindow()->Render();
		iren->Initialize();
		iren->Start();
		return;
	}*/

}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::SetLambda(double lambda)
{
	this->lambda = lambda;
	return;
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::InternalUpdate()
{
	// Check source, target

	double startTime, endTime, length;
	startTime = GetTickCount();

	if (this->Source == NULL || !this->Source->GetNumberOfPoints())
	{
		vtkErrorMacro(<<"Can't execute with NULL or empty input");
		return;
	}

	if (this->Target == NULL || !this->Target->GetNumberOfPoints())
	{
		vtkErrorMacro(<<"Can't execute with NULL or empty target");
		return;
	}

	// Create locator

	this->CreateDefaultLocator();
	this->Locator->SetDataSet(this->Target);
	this->Locator->SetNumberOfCellsPerBucket(1);
	this->Locator->BuildLocator();

	// Create two sets of points to handle iteration

	int step = 1;
	if (this->Source->GetNumberOfPoints() > this->MaximumNumberOfLandmarks)
	{
		step = this->Source->GetNumberOfPoints() / this->MaximumNumberOfLandmarks;
		//cout<<"step :"<<step<<endl;
		//cout<<"this->Source->GetNumberOfPoints() :::" <<this->Source->GetNumberOfPoints()<<endl;
		//cout<<"this->MaximumNumberOfLandmarks :::" <<this->MaximumNumberOfLandmarks<<endl;
		//vtkDebugMacro(<< "Landmarks step is now : " << step);
	}

	vtkIdType nb_points = this->Source->GetNumberOfPoints() / step;

	// Allocate some points.
	// - closestp is used so that the internal state of LandmarkTransform remains
	//   correct whenever the iteration process is stopped (hence its source
	//   and landmark points might be used in a vtkThinPlateSplineTransform).
	// - points2 could have been avoided, but do not ask me why 
	//   InternalTransformPoint is not working correctly on my computer when
	//   in and out are the same pointer.

	vtkPoints *points1 = vtkPoints::New();
	points1->SetNumberOfPoints(nb_points);

	vtkPoints *closestp = vtkPoints::New();
	closestp->SetNumberOfPoints(nb_points);

	vtkPoints *points2 = vtkPoints::New();
	points2->SetNumberOfPoints(nb_points);

	// Fill with initial positions (sample dataset using step)

	vtkTransform *accumulate = vtkTransform::New();
	accumulate->PostMultiply();

	vtkIdType i;
	int j;
	double p1[3], p2[3];

	if (StartByMatchingCentroids)
	{
		double source_centroid[3] = {0,0,0};
		for (i = 0; i < this->Source->GetNumberOfPoints(); i++)
		{
			this->Source->GetPoint(i, p1);
			source_centroid[0] += p1[0];
			source_centroid[1] += p1[1];
			source_centroid[2] += p1[2];
		}
		source_centroid[0] /= this->Source->GetNumberOfPoints();
		source_centroid[1] /= this->Source->GetNumberOfPoints();
		source_centroid[2] /= this->Source->GetNumberOfPoints();

		double target_centroid[3] = {0,0,0};
		for (i = 0; i < this->Target->GetNumberOfPoints(); i++)
		{
			this->Target->GetPoint(i, p2);
			target_centroid[0] += p2[0];
			target_centroid[1] += p2[1];
			target_centroid[2] += p2[2];
		}
		target_centroid[0] /= this->Target->GetNumberOfPoints();
		target_centroid[1] /= this->Target->GetNumberOfPoints();
		target_centroid[2] /= this->Target->GetNumberOfPoints();

		accumulate->Translate(target_centroid[0] - source_centroid[0],
			target_centroid[1] - source_centroid[1],
			target_centroid[2] - source_centroid[2]);
		accumulate->Update();

		for (i = 0, j = 0; i < nb_points; i++, j += step)
		{
			double outPoint[3];
			accumulate->InternalTransformPoint(this->Source->GetPoint(j),
				outPoint);
			points1->SetPoint(i, outPoint);
		}
	}
	else 
	{
		for (i = 0, j = 0; i < nb_points; i++, j += step)
		{
			points1->SetPoint(i, this->Source->GetPoint(j));
		}
	}

	// Go
	vtkIdType cell_id;
	int sub_id;
	double dist2, totaldist = 0;
	double outPoint[3];
	double f1 = 1,f0 = 1;
	
	vtkPoints *temp, *a = points1, *b = points2;

	this->NumberOfIterations = 0;

	DISTELEM* distList=new DISTELEM[nb_points];

	vtkPoints *af;
	vtkPoints *cf;

	double preMeanDistance = -100.0;

	unsigned char whiteTuple[3] = {255,255,255};
	unsigned char redTuple[3]   = {255, 0 , 0 };
	unsigned char blueTuple[3]  = {0,   0 ,255};
	unsigned char blackTuple[3] = {0 ,  0 , 0 };
	//unsigned char normalTuple[3]= {255,255,255};
	unsigned char normalTuple[3]= { 255 , 255 , 255 };
	double pointIn = 1;
	double pointOut = 0;
	 
	vtkSmartPointer<vtkUnsignedCharArray> distArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
	distArray->SetNumberOfComponents(3);
	distArray->SetName("RGB");

	vtkDoubleArray * InOrOutArray = vtkDoubleArray::New();
	InOrOutArray->SetNumberOfComponents(1);
	InOrOutArray->SetName("InOrOut");
	
	//distArray->PrintSelf(cout,vtkIndent(1));
	for (i = 0; i< this->Source->GetNumberOfPoints() ;i++)
	{// set the points to be normal. 
		distArray->InsertNextTupleValue( normalTuple);
		InOrOutArray->InsertNextTupleValue(&pointIn);				// 1 means in
	}

	//this->Source->GetPointData()->PrintSelf(cout,vtkIndent(1));
	this->Source->GetPointData()->SetScalars(distArray);
	this->Source->GetPointData()->AddArray(InOrOutArray);
	//this->Source->GetPointData()->PrintSelf(cout,vtkIndent(1));

	do 
	{
		af = vtkPoints::New();
		cf = vtkPoints::New();

		// cout<<"\r It: "<<this->NumberOfIterations<<" C: "<<this->MeanDistance;
		// Fill points with the closest points to each vertex in input

		for(i = 0; i < nb_points; i++)
		{
			this->Locator->FindClosestPoint( a->GetPoint(i), outPoint, 
				cell_id, sub_id, dist2);
			closestp->SetPoint(i, outPoint);

			distList[i].id = i;
			distList[i].dist = dist2;
		}

		qsort(distList,nb_points,sizeof(DISTELEM),compare);
		distList[0].accum = distList[0].dist;
		for(i = 1; i < nb_points; i++)
		{
			distList[i].accum = distList[i-1].accum + distList[i].dist;
		}

		// cout<<" Dist: "<<distList[nb_points-1].accum/nb_points;

		int minId = (int)(nb_points/10);
		double minDist = pow((minId+1.0)/nb_points,-this->lambda)*sqrt(distList[minId].accum/(minId+1));

		// Find the optimal f;
		for(i = (int)(nb_points/10); i < nb_points; i++)
		{
			distList[i].accum = pow((i+1.0)/nb_points,-this->lambda)*sqrt(distList[i].accum/(i+1));
			if(distList[i].accum<=minDist){
				minDist = distList[i].accum;
				minId = i;	
			}
		}

		// Output the f
		f1 = double(minId+1)/nb_points;
		// cout<<" f: "<<f1<<" DistF: "<<distList[minId].accum/(minId+1);
		if(distList[minId].dist == -1)
			distList[minId].dist = -1;

		// Copy the first minId+1 nodes to af and cf
		af->SetNumberOfPoints((vtkIdType)(minId+1));
		cf->SetNumberOfPoints((vtkIdType)(minId+1));

		for(i=0;i<=minId;i++){
			af->SetPoint(i,a->GetPoint(distList[i].id));
			cf->SetPoint(i,closestp->GetPoint(distList[i].id));	
		}

		//distArray->PrintSelf(cout,vtkIndent(1));
		/*distArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
		distArray->SetNumberOfComponents(3);
		distArray->SetName("RGB");*/

		for (i = 0; i< this->Source->GetNumberOfPoints() ;i++){	
			distArray->SetTupleValue(i, normalTuple);
			InOrOutArray->SetTupleValue(i,&pointIn);
		}

		// set the points in Df to red
		for(i=0;i<=minId;i++){
			for(int j=distList[i].id*step; j <(distList[i].id+1)*step;j++)
			{
				distArray->SetTupleValue(j,redTuple);
				InOrOutArray->SetTupleValue(j,&pointIn);
			}
		}
		// set the points out Df to blue
		for(i=minId+1;i<nb_points;i++){
			for(int j=distList[i].id*step; j <(distList[i].id+1)*step;j++)
			{
				distArray->SetTupleValue(j,blueTuple);
				InOrOutArray->SetTupleValue(j,&pointOut);
			}
		}
		this->Source->GetPointData()->SetScalars(distArray);

		// Build the landmark transform
		this->LandmarkTransform->SetSourceLandmarks(af);
		this->LandmarkTransform->SetTargetLandmarks(cf);
		this->LandmarkTransform->Update();

		// Concatenate (can't use this->Concatenate directly)
		accumulate->Concatenate(this->LandmarkTransform->GetMatrix());

		if(VisualizeOn)
			Visualize(accumulate,0);

		this->NumberOfIterations++;
		vtkDebugMacro(<< "Iteration: " << this->NumberOfIterations);
		if (this->NumberOfIterations >= this->MaximumNumberOfIterations) 
		{
			endTime = GetTickCount();
			length = endTime - startTime;
			cout<<"Mean distance criteria reached.";
			cout<<" It: "<<this->NumberOfIterations;
			cout<<" C: "<<this->MeanDistance;
			// cout<<" Dist: "<<distList[nb_points-1].accum/nb_points;
			cout<<" f: "<<f1;
			// cout<<" DistF: "<<distList[minId].accum/(minId+1);
			cout<<" Cost: "<<length/1000.0<<" seconds";
			cout<<endl;
			break;
		}

		// Move mesh and compute mean distance if needed

		if (this->CheckMeanDistance)
		{
			totaldist = 0.0;
		}

		for(i = 0; i < nb_points; i++)
		{
			a->GetPoint(i, p1);
			this->LandmarkTransform->InternalTransformPoint(p1, p2);
			b->SetPoint(i, p2);
			if (this->CheckMeanDistance)
			{
				if (this->MeanDistanceMode == VTK_ICP_MODE_RMS) 
				{
					totaldist += vtkMath::Distance2BetweenPoints(p1, p2);
				} else {
					totaldist += sqrt(vtkMath::Distance2BetweenPoints(p1, p2));
				}
			}
		}

		if (this->CheckMeanDistance)
		{
			if (this->MeanDistanceMode == VTK_ICP_MODE_RMS) 
			{
				this->MeanDistance = sqrt(totaldist / (double)nb_points);
			} else {
				this->MeanDistance = totaldist / (double)nb_points;
			}
			vtkDebugMacro("Mean distance: " << this->MeanDistance);
			if (this->MeanDistance <= this->MaximumMeanDistance )// || abs(this->MeanDistance - preMeanDistance) <= this->MaximumMeanDistance/30.0)
			{
				endTime = GetTickCount();
				length = endTime - startTime;
				cout<<"Mean distance criteria reached.";
				cout<<" It: "<<this->NumberOfIterations;
				cout<<" C: "<<this->MeanDistance;
				// cout<<" Dist: "<<distList[nb_points-1].accum/nb_points;
				cout<<" f: "<<f1;
				// cout<<" DistF: "<<distList[minId].accum/(minId+1);
				cout<<" Cost: "<<length/1000.0<<" seconds";
				cout<<endl;
				break;
			}
			preMeanDistance = this->MeanDistance;
		}

		temp = a;
		a = b;
		b = temp;

		//same = 1;
		//for(i = 0; i < nb_points; i++)
		//{
		//	this->Locator->FindClosestPoint(a->GetPoint(i), outPoint, cell_id, sub_id, dist2);
		//	closestp->GetPoint(i,p3);
		//	if(p3[0] != outPoint[0] || p3[1] != outPoint[1] || p3[2] != outPoint[2])
		//	{
		//		same = 0;
		//	}
		//}
		//if (same == 1 && f1 == f0)
		//{
		//	break;
		//}

		//f0 = f1;

		af->Delete();
		cf->Delete();
	} 
	while (1);

	// cout<<endl<<endl<<"FICP completed. drag to see the result."<<endl;

	if(VisualizeOn)
		Visualize(accumulate,1);

	// Now recover accumulated result

	this->Matrix->DeepCopy(accumulate->GetMatrix());

	accumulate->Delete();
	points1->Delete();
	closestp->Delete();
	points2->Delete();
}

//----------------------------------------------------------------------------

void vtkFractionalIterativeClosestPointTransform::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os,indent);

	if ( this->Source ) 
	{
		os << indent << "Source: " << this->Source << "\n";
	}
	else 
	{
		os << indent << "Source: (none)\n";
	}

	if ( this->Target ) 
	{
		os << indent << "Target: " << this->Target << "\n";
	}
	else 
	{
		os << indent << "Target: (none)\n";
	}

	if ( this->Locator ) 
	{
		os << indent << "Locator: " << this->Locator << "\n";
	}
	else 
	{
		os << indent << "Locator: (none)\n";
	}

	os << indent << "MaximumNumberOfIterations: " << this->MaximumNumberOfIterations << "\n";
	os << indent << "CheckMeanDistance: " << this->CheckMeanDistance << "\n";
	os << indent << "MeanDistanceMode: " << this->GetMeanDistanceModeAsString() << "\n";
	os << indent << "MaximumMeanDistance: " << this->MaximumMeanDistance << "\n";
	os << indent << "MaximumNumberOfLandmarks: " << this->MaximumNumberOfLandmarks << "\n";
	os << indent << "StartByMatchingCentroids: " << this->StartByMatchingCentroids << "\n";
	os << indent << "NumberOfIterations: " << this->NumberOfIterations << "\n";
	os << indent << "MeanDistance: " << this->MeanDistance << "\n";
	if(this->LandmarkTransform)
	{
		os << indent << "LandmarkTransform:\n";
		this->LandmarkTransform->PrintSelf(os, indent.GetNextIndent());
	}
}


void vtkFractionalIterativeClosestPointTransform::SetVisualizeOn()
{
	VisualizeOn = 1;
}

void vtkFractionalIterativeClosestPointTransform::SetVisualizeOff()
{
	VisualizeOn = 0;
}

void vtkFractionalIterativeClosestPointTransform::SetVisualizeRenderer(vtkRenderer *ren)
{
	this->Ren = ren;
}
