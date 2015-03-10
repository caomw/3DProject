#include "FICPRegister.h"


int FICPRegister(string ref,string target,string savePath)
{
	vtkPLYReader *reader_ref = vtkPLYReader::New();
	reader_ref->SetFileName(ref.c_str());
	reader_ref->Update();

	vtkPLYReader *reader_tgt = vtkPLYReader::New();
	reader_tgt->SetFileName(target.c_str());
	reader_tgt->Update();

	vtkDelaunay3D *ref_d = vtkDelaunay3D::New();
	ref_d->SetInput(reader_ref->GetOutput());
	ref_d->Update();

	vtkDelaunay3D *tgt_d = vtkDelaunay3D::New();
	tgt_d->SetInput(reader_tgt->GetOutput());
	tgt_d->Update();

	vtkGeometryFilter *ref_Filter = vtkGeometryFilter::New();
	ref_Filter->SetInput(ref_d->GetOutput());
	ref_Filter->Update();
	vtkGeometryFilter *tgt_Filter = vtkGeometryFilter::New();
	tgt_Filter->SetInput(tgt_d->GetOutput());
	tgt_Filter->Update();

	vtkFractionalIterativeClosestPointTransform *ficp = vtkFractionalIterativeClosestPointTransform::New();
	ficp->SetSource(ref_Filter->GetOutput());
	ficp->SetTarget(tgt_Filter->GetOutput());
	ficp->SetLambda(2.8);
	ficp->SetVisualizeOff();
	ficp->Update();

	vtkTransformPolyDataFilter *transform_ficp = vtkTransformPolyDataFilter::New();
	transform_ficp->SetInput(reader_ref->GetOutput());
	transform_ficp->SetTransform(ficp);
	transform_ficp->Update();

	vtkMatrix4x4* vtk = ficp->GetMatrix();
	cout<<*vtk<<endl;

	vtkPLYWriter *writer = vtkPLYWriter::New();
	writer->SetInput(transform_ficp->GetOutput());
	writer->SetFileName(savePath.c_str());
	writer->Write();
	return 0;
}
