#ifndef __FeetSegmentation_h
#define __FeetSegmentation_h

// Qt Includes
#include <QString>
#include <QImage>

// MRML Includes
#include <vtkMRMLVectorVolumeNode.h>

#include <Torch>

class FeetSegmentation
{
public:
  explicit FeetSegmentation();

  std::vector<QImage> predict(QString datasetDir, size_t batchSize=4);
  std::vector<vtkImageData *> predict(vtkMRMLVectorVolumeNode *datasetNode, size_t batchSize=4);
//  void predict(QString outputDir, QString datasetDir, size_t batchSize=4);

private:
  torch::Device device;
  torch::jit::script::Module model;
  torch::data::transforms::Normalize<> normalize;

  int loadModel(QString modelFile);
};

#endif // __FeetSegmentation_h
