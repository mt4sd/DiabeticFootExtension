#ifndef VTKFEETSEGMENTATIONDEPTHDATASET_H
#define VTKFEETSEGMENTATIONDEPTHDATASET_H

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// Eigen Includes
#include <Eigen/Geometry>

// VTK Includes
#include <vtkImageData.h>

// Qt Includes
#include <QSize>

class vtkFeetSegmentationDepthDataset
{
public:
  vtkFeetSegmentationDepthDataset(vtkImageData *depthImg, QSize resize=QSize(0,0));
  ~vtkFeetSegmentationDepthDataset();

  /**
   * @brief applyMask
   * @param mask
   */
  void applyMask(vtkImageData *mask);

  /**
   * @brief getPointCloud
   * @return
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const { return pc; }

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
   * @brief setInliers
   * @param indices
   */
  void setInliers(pcl::PointIndices::Ptr indices);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
  vtkImageData *depthImg;

  vtkImageData *maskResult;

  /**
   * @brief img2pc: Transform applied for a correct visualization of the point cloud
   */
  Eigen::Affine3f img2pc;

  /**
   * @brief generatePointCloud: Reading the grayscale 16 bit png, generate a point cloud.
   * @param img: vtkImagaData in format Grayscale 16 bit.
   */
  void generatePointCloud(vtkImageData *img);
};

#endif // VTKFEETSEGMENTATIONDEPTHDATASET_H
