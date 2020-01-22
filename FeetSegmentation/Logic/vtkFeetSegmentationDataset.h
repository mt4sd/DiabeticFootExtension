#ifndef __vtkFeetSegmentationDataset_h
#define __vtkFeetSegmentationDataset_h

// Qt Includes
#include <QSize>

// VTK Includes
#include <vtkImageData.h>

// Torch Includes
#include <Torch>

// MRMLM Includes
#include "vtkMRMLVectorVolumeNode.h"

class vtkFeetSegmentationDataset : public torch::data::Dataset<vtkFeetSegmentationDataset>
{
public:
  vtkFeetSegmentationDataset(vtkMRMLVectorVolumeNode *inputData);

  // Override the get method to load custom data.
  torch::data::Example<> get(size_t index) override;

  // Override the size method to infer the size of the data set.
  torch::optional<size_t> size() const override { return data->GetImageData()->GetDimensions()[2]; }

private:
  vtkMRMLVectorVolumeNode *data;
  QSize shape;
};

#endif // __vtkFeetSegmentationDataset_h
