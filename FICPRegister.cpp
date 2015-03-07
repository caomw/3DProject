#include "FICPRegister.h"

int FICPRegister(string ref,string target,string savePath)
{
	vtkPLYReader *reader_ref = vtkPLYReader::New();
	reader_ref->SetFileName(ref.c_str());
	reader_ref->Update();
	vtkPLYReader *reader_tgt = vtkPLYReader::New();
	reader_tgt->SetFileName(target.c_str());
	reader_tgt->Update();
	vtkFractionalIterativeClosestPointTransform *ficp = vtkFractionalIterativeClosestPointTransform::New();
	ficp->SetSource(reader_ref->GetOutput());
	ficp->SetTarget(reader_tgt->GetOutput());
	ficp->SetLambda(2.8);
	ficp->SetVisualizeOff();
	ficp->Update();

	vtkTransformPolyDataFilter *transform_ficp = vtkTransformPolyDataFilter::New();
	transform_ficp->SetInput(reader_ref->GetOutput());
	transform_ficp->SetTransform(ficp);
	transform_ficp->Update();

	vtkPLYWriter *writer = vtkPLYWriter::New();
	writer->SetInput(transform_ficp->GetOutput());
	writer->SetFileName(savePath.c_str());
	writer->Write();
	return 0;
}
