#ifndef __FeetSegmentationDataset_h
#define __FeetSegmentationDataset_h

#include <Torch>

// Qt Includes
#include <QStringList>
#include <QSize>

// MRMLM Includes
#include "vtkMRMLVectorVolumeNode.h"

class FeetSegmentationDataset : public torch::data::Dataset<FeetSegmentationDataset>
{
public:
  FeetSegmentationDataset(QString &imgDir);

  // Override the get method to load custom data.
  torch::data::Example<> get(size_t index) override;

  // Override the size method to infer the size of the data set.
  torch::optional<size_t> size() const override { return imgLocation.size(); }

private:
  QStringList imgLocation;
  QSize shape;

  QStringList readData(const QString &imgDir);
};

#endif // __FeetSegmentationDataset_h
