#include "Utils.h"

// VTK Includes
#include <vtkSmartPointer.h>

// Temporal Includes
#include <QDebug>

torch::Tensor Utils::qImageToTensor(QImage &img)
{
    if (img.format() != QImage::Format_RGB888)
        img = img.convertToFormat(QImage::Format_RGB888);

    int nPixels = img.size().rheight()*img.size().rwidth()*3;
    torch::Tensor tmpTensor = torch::from_blob(img.bits(), {nPixels},
                                                torch::kByte).clone();

    torch::Tensor imgTensor = torch::zeros({3, img.size().rheight(), img.size().rwidth()});
    imgTensor.slice(0, 0, 1) = tmpTensor.slice(0, 0, img.size().rheight()*img.size().rwidth()*3, 3).reshape({1, img.size().rheight(), img.size().rwidth()});
    imgTensor.slice(0, 1, 2) = tmpTensor.slice(0, 1, img.size().rheight()*img.size().rwidth()*3, 3).reshape({1, img.size().rheight(), img.size().rwidth()});
    imgTensor.slice(0, 2, 3) = tmpTensor.slice(0, 2, img.size().rheight()*img.size().rwidth()*3, 3).reshape({1, img.size().rheight(), img.size().rwidth()});

    imgTensor = imgTensor.to(torch::kFloat32);
    imgTensor /= 255;

    return imgTensor;
}

torch::Tensor Utils::vtkImageToTensor(vtkImageData *img)
{
  int *imgSize = img->GetDimensions();

  int nPixels = imgSize[0]*imgSize[1]*img->GetNumberOfScalarComponents();
  uint8_t *bits = reinterpret_cast<uint8_t *>(img->GetScalarPointer());

  torch::Tensor tmpTensor = torch::from_blob(bits, {nPixels},
                                              torch::kByte).clone();

  torch::Tensor imgTensor = torch::zeros({3, imgSize[1], imgSize[0]});
  imgTensor.slice(0, 0, 1) = tmpTensor.slice(0, 0, nPixels, 3).reshape({1, imgSize[1], imgSize[0]});
  imgTensor.slice(0, 1, 2) = tmpTensor.slice(0, 1, nPixels, 3).reshape({1, imgSize[1], imgSize[0]});
  imgTensor.slice(0, 2, 3) = tmpTensor.slice(0, 2, nPixels, 3).reshape({1, imgSize[1], imgSize[0]});

  imgTensor = imgTensor.to(torch::kFloat32);
  imgTensor /= 255;

  return imgTensor;
}

QImage Utils::tensorToQImage(torch::Tensor &tensor, QImage::Format format){
    torch::Tensor _tensor = tensor.permute({1, 2, 0});
    _tensor = _tensor.mul(255).clamp(0, 255).to(torch::kU8);
    QImage img = QImage(_tensor.size(1), _tensor.size(0), format);

    _tensor = _tensor.flatten();
    std::memcpy((void *) img.bits(), _tensor.data_ptr(), sizeof(torch::kU8) * _tensor.numel());

    img.save("vtkPrueba.png");

    return img;
}

vtkImageData* Utils::tensorToVtkImage(torch::Tensor &tensor)
{
  torch::Tensor _tensor = tensor.permute({1, 2, 0});
  _tensor = _tensor.mul(255).clamp(0, 255).to(torch::kU8);

  vtkImageData *img = vtkImageData::New();

  img->SetDimensions(_tensor.size(1), _tensor.size(0), 1);
  img->SetSpacing(1.0, 1.0, 1.0);
  img->SetOrigin(.0, .0, .0);

  img->AllocateScalars(VTK_UNSIGNED_CHAR, 1);

  std::memcpy(img->GetScalarPointer(), _tensor.data_ptr(), sizeof(torch::kU8) * _tensor.numel());

  return img;
}

QStringList Utils::absoluteDir(QDir root, QStringList filenames)
{
  QString absoluteDir = root.absolutePath();
  for (QStringList::Iterator fileIt = filenames.begin(); fileIt != filenames.end(); ++fileIt)
  {
    *fileIt = absoluteDir + "/" + *fileIt;
  }

  return filenames;
}

//----------------------------------------------------------
torch::Tensor Utils::binarize(torch::Tensor &tensor, double threshold)
{
    if (threshold < 0 || threshold > 1)
        throw std::invalid_argument("threshold values must be in the range [0, 1]");

    return torch::where(tensor >= threshold, torch::ones(1), torch::zeros(1));
}

//----------------------------------------------------------
#include <pcl/io/pcd_io.h>
// To remove
pcl::PointCloud<pcl::PointXYZ> Utils::vtkImageToPointCloud(vtkImageData *depthImg)
{
  qDebug() << "Aqué estoy!...";
  uint16_t *data = reinterpret_cast<uint16_t *>(depthImg->GetScalarPointer());
  int *dimensions = depthImg->GetDimensions();

  pcl::PointCloud<pcl::PointXYZ> pointCloud;

  qDebug() << "Voy a entrar a los bucles, deseame suerte!...";
  for (int x = 0; x < dimensions[0]; ++x)
  {
    for (int y=0; y < dimensions[1]; ++y)
    {
      uint16_t depthPixel = data[ y * dimensions[0] + x];
      pointCloud.push_back(pcl::PointXYZ(x, y, depthPixel));
    }
  }

  //tmp
  pcl::io::savePCDFile("test.pcd", pointCloud);
  return pointCloud;
}

//----------------------------------------------------------
vtkImageData * Utils::pointCloudToVtkImage(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{

}
