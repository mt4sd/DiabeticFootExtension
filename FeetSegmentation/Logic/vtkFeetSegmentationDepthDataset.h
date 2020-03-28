#ifndef VTKFEETSEGMENTATIONDEPTHDATASET_H
#define VTKFEETSEGMENTATIONDEPTHDATASET_H

// PCL Includes
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// Eigen Includes
#include <Eigen/Geometry>

// VTK Includes
#include <vtkImageData.h>

// Qt Includes
#include <QSize>

// Std Includes
#include <vector>

namespace test {

/**
 * @brief This class contains the depth pixel values from a images and a representation as point cloud where
 *  the points contains the X, Y and Z components.
 */
class vtkFeetSegmentationDepthDataset
{
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;
public:
  vtkFeetSegmentationDepthDataset(vtkImageData *depthImg, QSize resize=QSize(0,0));
  ~vtkFeetSegmentationDepthDataset();

  /**
   * @brief getPointCloud
   * @return
   */
  PointCloud::Ptr getPointCloud() const { return cloud; }

  /**
   * @brief getDepthImage
   * @return
   */
  vtkImageData * getDepthImage() const { return depthImg; }

  /**
   * @brief getMaskResult
   * @return
   */
  vtkImageData * getMaskResult() const { return maskResult; }

  /**
   * @brief Apply a mask to the depth pixel values, this function update the
   *  Point Cloud representation.
   * @param vtkImageData with binary values.
   */
  void applyMask(vtkImageData *mask);

  /**
   * @brief Extract the inliers in the point cloud and update the vtkImageData
   *  representation
   * @param A vector with the indices of the inliers within the point cloud.
   */
  void setInliers(std::vector<int> indices);

private:
  vtkImageData *depthImg;
  PointCloud::Ptr cloud;

  vtkImageData *maskResult;

  /**
   * @brief img2pc: Transform applied for a correct visualization of the point cloud
   */
  Eigen::Affine3f img2pc;

  /**
   * @brief generatePointCloud: Reading the grayscale 16 bit png, generate a point cloud.
   * @param vtkImagaData in Grayscale 16 bit format.
   */
  void vtkImageToPointCloud(vtkImageData *img);

  /**
   * @brief pointCloudToVtkImageData: Generate a grayscale 16 bit image from a point cloud.
   *    In this image, the grayscale pixel value corresponds to the depth value
   *    (Z component in the point cloud).
   * @param pointCloud: Point Cloud with point in XYZ format.
   */
  void pointCloudToVtkImage(PointCloud::Ptr pointCloud);
};

}

#endif // VTKFEETSEGMENTATIONDEPTHDATASET_H
