#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <QVTKWidget.h>
#include <QApplication>

#include "vtkRenderWindow.h"


int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  QVTKWidget widget;
  widget.resize(512, 256);

  //
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    {

      for (float y = -0.5f; y <= 0.5f; y += 0.01f)
      {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f)
        {
          pcl::PointXYZ point;
          point.x = 2.0f - y;
          point.y = y;
          point.z = z;
          cloud_xyz->points.push_back (point);
        }
      }
      cloud_xyz->width = cloud_xyz->points.size ();
      cloud_xyz->height = 1;

    }

    // this creates and displays a window named "test_viz"
    // upon calling PCLVisualizerInteractor interactor_->Initialize ();
    // how to disable that?
    pcl::visualization::PCLVisualizer pviz ("test_viz");

    pviz.addPointCloud<pcl::PointXYZ>(cloud_xyz);
    pviz.setBackgroundColor(0, 0, 0.1);

    vtkSmartPointer<vtkRenderWindow> renderWindow = pviz.getRenderWindow();
    widget.SetRenderWindow(renderWindow);
  }

  widget.show();
  app.exec();

  return EXIT_SUCCESS;
}
