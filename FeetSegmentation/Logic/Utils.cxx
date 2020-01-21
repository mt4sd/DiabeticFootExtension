#include "Utils.h"

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

QImage Utils::tensorToQImage(torch::Tensor &tensor, QImage::Format format){
    torch::Tensor _tensor = tensor.permute({1, 2, 0});
    _tensor = _tensor.mul(255).clamp(0, 255).to(torch::kU8);
    QImage img = QImage(_tensor.size(1), _tensor.size(0), format);

    _tensor = _tensor.flatten();
    std::memcpy((void *) img.bits(), _tensor.data_ptr(), sizeof(torch::kU8) * _tensor.numel());

    img.save("prueba.png");

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

torch::Tensor Utils::binarize(torch::Tensor &tensor, double threshold)
{
    if (threshold < 0 || threshold > 1)
        throw std::invalid_argument("threshold values must be in the range [0, 1]");

    return torch::where(tensor >= threshold, torch::ones(1), torch::zeros(1));
}
