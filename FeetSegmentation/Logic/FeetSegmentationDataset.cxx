#include "FeetSegmentationDataset.h"
#include "Utils.h"

#include <QDir>

// Temp
#include <QImage>

FeetSegmentationDataset::FeetSegmentationDataset(QString &imgDir)
  : imgLocation(readData(imgDir)), shape(512,512)
{}

torch::data::Example<> FeetSegmentationDataset::get(size_t index)
{
  QImage img(imgLocation[static_cast<int>(index)]);
  img = img.scaled(shape, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

  torch::Tensor imgTensor = Utils::qImageToTensor(img);

  return {imgTensor, torch::zeros(imgTensor.sizes())};
}

QStringList FeetSegmentationDataset::readData(const QString &img_dir)
{
    QStringList filenames;

    QDir rootDir = QDir(img_dir);
    if (!rootDir.exists())
     return filenames;

    QStringList nameFilter;
    nameFilter << "*.png" << "*.jpg";

    filenames = rootDir.entryList(nameFilter);
    filenames = Utils::absoluteDir(rootDir, filenames);

    return filenames;
}
