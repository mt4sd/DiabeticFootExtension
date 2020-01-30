#ifndef __Utils_h
#define __Utils_h

#include <QImage>
#include <QDir>
#include <Torch>

// VTK Includes
#include <vtkImageData.h>

// PCL Include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Utils
{
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
public:
  /**
   * @brief qImageToTensor
   * @param img
   * @return
   */
  static torch::Tensor qImageToTensor(QImage &img);

  /**
   * @brief vtkImageToTensor
   * @param img
   * @return
   */
  static torch::Tensor vtkImageToTensor(vtkImageData *img);

  /**
   * @brief tensorToQImage
   * @param tensor
   * @param format
   * @return
   */
  static QImage tensorToQImage(torch::Tensor &tensor, QImage::Format format);

  /**
   * @brief tensorToVtkImage
   * @param tensor
   * @return
   */
  static vtkImageData* tensorToVtkImage(torch::Tensor &tensor);

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

  /**
   * @brief vtkImateToPointCloud
   * @param img
   * @return
   */
  static pcl::PointCloud<pcl::PointXYZ> vtkImageToPointCloud(vtkImageData *img);

  /**
   * @brief vtkImateToPointCloud2
   * @param img
   * @return
   */
  static PointCloud::Ptr vtkImageToPointCloud2(vtkImageData *img);

  /**
   * @brief pointCloudToVtkImage
   * @param pointCloud
   * @return
   */
  static vtkImageData * pointCloudToVtkImage(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
};

#endif // UTILS_H
