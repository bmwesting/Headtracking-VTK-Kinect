#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkCylinderSource.h>
#include <vtkCommand.h>
#include <vtkDataSetReader.h>
#include <vtkContourFilter.h>
#include <vtkSphereSource.h>

#include "SensorDevice.h"
#include "SkeletonMath.h"

// globals
SensorDevice* sensor;
vtkSmartPointer<vtkRenderer> ren;
vtkSmartPointer<vtkRenderWindow> renWin;

void updateCamera(Point& pt)
{

    // check for boundary conditions (kinect specific)
    if(pt.x_ < 0) pt.x_ = 0;
    if(pt.x_ >640) pt.x_ = 640;
    if(pt.y_ < 0) pt.y_ = 0;
    if(pt.y_ > 480) pt.y_ = 480;

    // x ranges from -320, 320
    float xLoc = pt.x_ - 320;
    
    // y ranges from -240, 240
    float yLoc = pt.y_ - 240;
    
    // z
    float zLoc = pt.z_;
    
    // scale factor
    float factor = .03;
    
    ren->GetActiveCamera()->SetViewShear(factor*xLoc/zLoc, -factor*yLoc/zLoc, zLoc*factor);
    
    //printf("degree change: %f, %f.\n", degreeChangeX, degreeChangeY);
    
    renWin->Render();

}

class UpdateData:public vtkCommand
{
	public:
		static UpdateData *New()
		{
			UpdateData* up = new UpdateData;
			return up;
		}
		virtual void Execute(vtkObject* caller, unsigned long eid, void* clientdata)
		{
			sensor->waitForDeviceUpdateOnUser();
			if(sensor->isTracking())
			{
			    Point pt;
			    sensor->getHeadPoint(sensor->getUID(0), &pt);
			    
			    // if we have confidence in the location of user's head
			    if(pt.confidence_ == 1.0)
			        updateCamera(pt);
			}
		}
};

int main(int argc, char** argv)
{

    // initialize vtk pipeline
	ren = vtkSmartPointer<vtkRenderer>::New();
	renWin = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renWin->AddRenderer(ren);
	renWin->SetInteractor(iren);
	renWin->SetSize(1024,768);
	
	// mummy
	vtkSmartPointer<vtkDataSetReader> data = vtkSmartPointer<vtkDataSetReader>::New();
	vtkSmartPointer<vtkContourFilter> contour = vtkSmartPointer<vtkContourFilter>::New();
	vtkSmartPointer<vtkPolyDataMapper> cMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> cActor = vtkSmartPointer<vtkActor>::New();
	cMapper->SetInput(contour->GetOutput());
	cActor->SetMapper(cMapper);
	ren->AddActor(cActor);
	
	// isosurfaces
	data->SetFileName("mummy.128.vtk");
	contour->SetInput(data->GetOutput());
	contour->SetValue(0, 125);
	
	// sphere
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	vtkSmartPointer<vtkPolyDataMapper> sMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> sActor = vtkSmartPointer<vtkActor>::New();
	sMapper->SetInput(sphere->GetOutput());
	sActor->SetMapper(sMapper);
	ren->AddActor(sActor);
	sphere->SetCenter(50, 500, 0);
	sphere->SetRadius(50);
	
	
	//Add timer callback
	vtkSmartPointer<UpdateData> updateCallback = vtkSmartPointer<UpdateData>::New();
	iren->AddObserver(vtkCommand::TimerEvent, updateCallback);
	iren->CreateRepeatingTimer(100);
	
	// initialize the kinect
	sensor = new SensorDevice();
	sensor->initialize();
	sensor->startGeneratingAll();
	sensor->lookForCalibrationPoseOn();
		
	// initial position of center of camera
	lastPoint.x_ = 320;
	lastPoint.y_ = 240;
	lastPoint.z_ = 0;
	
	iren->Initialize();
	iren->Start();
	
	delete sensor;
	
    return 0;
}