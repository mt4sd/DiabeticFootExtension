#ifndef __Utils_h
#define __Utils_h

#include <QImage>
#include <QDir>
#include <Torch>

// VTK Includes
#include <vtkImageData.h>

class Utils
{
public:
  /**
   * @brief qImageToTensor
   * @param img
   * @return
   */
  static torch::Tensor qImageToTensor(QImage &img);

  static torch::Tensor vtkImageToTensor(vtkImageData *img);

  /**
   * @brief tensorToQImage
   * @param tensor
   * @param format
   * @return
   */
  static QImage tensorToQImage(torch::Tensor &tensor, QImage::Format format);

  /**
   * @brief absoluteDir
   * @param root
   * @param filenames
   * @return
   */
  static QStringList absoluteDir(QDir root, QStringList filenames);

  /**
   * @brief binarize
   * @param tensor
   * @param threshold
   * @return
   */
  static torch::Tensor binarize(torch::Tensor &tensor, double threshold=0.75);
};

#endif // UTILS_H
