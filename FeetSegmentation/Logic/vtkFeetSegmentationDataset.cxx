#include "vtkFeetSegmentationDataset.h"
#include "Utils.h"

// VTK Includes
#include <vtkImageResize.h>

namespace test{

vtkFeetSegmentationDataset::vtkFeetSegmentationDataset(vtkMRMLVectorVolumeNode *inputData)
  : data(inputData), shape(QSize(512,512))
{}

torch::data::Example<> vtkFeetSegmentationDataset::get(size_t index)
{
  vtkSmartPointer<vtkImageResize> resize =
      vtkSmartPointer<vtkImageResize>::New();

  //TODO: get a slicer from a vtkImageData, in this case Im working with just one simple image
  vtkImageData * img = data->GetImageData();

  //resize image
  resize->SetInputData(data->GetImageData());
  resize->SetOutputDimensions(shape.width(), shape.height(), 1);
  resize->Update();

  vtkImageData *resizedImg = resize->GetOutput();
  torch::Tensor imgTensor = Utils::vtkImageToTensor(resizedImg);

  return {imgTensor, torch::zeros(imgTensor.sizes())};
}

}