//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------



#include <mico/flow/blocks/BlockPointCloudVisualizer.h>
#include <mico/flow/policies/policies.h>

#include <mico/base/map3d/DataFrame.h>

#include <Eigen/Eigen>
#include <vtkInteractorStyleFlight.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointData.h>

namespace mico{

    BlockPointCloudVisualizer::BlockPointCloudVisualizer(){

        
        // Setup render window, renderer, and interactor
        renderWindow->SetWindowName("Pointcloud Visualization");
        renderWindow->AddRenderer(renderer);
        renderWindowInteractor->SetRenderWindow(renderWindow);
        renderer->SetBackground(colors->GetColor3d("Gray").GetData());

        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        widgetCoordinates_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        widgetCoordinates_->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
        widgetCoordinates_->SetOrientationMarker( axes );
        widgetCoordinates_->SetInteractor( renderWindowInteractor );
        widgetCoordinates_->SetViewport( 0.0, 0.0, 0.4, 0.4 );
        widgetCoordinates_->SetEnabled( 1 );

        // Visualize
        interactorThread_ = std::thread([&](){
            renderWindowInteractor->Start();
        });

        callback_ = [&](std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid){
            if(idle_){
                idle_ = false;
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = nullptr;
                if(_valid["cloud"]){
                    cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(_data["cloud"]); 
                }else if(_valid["dataframe"]){
                    std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> df = std::any_cast<std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>>>(_data["dataframe"]);
                    cloud = df->cloud;
                }

                if(cloud){
                    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
                    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
                    colors->SetNumberOfComponents(3);
                    colors->SetName ("Colors");
                    for(auto &p: *cloud){
                        points->InsertNextPoint (p.x, p.y, p.z);
                        unsigned char c[3] = {p.r, p.g, p.b};
                        colors->InsertNextTupleValue(c);
                    }

                    vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
                    pointsPolydata->SetPoints(points);
                    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
                    vertexFilter->SetInputData(pointsPolydata);
                    vertexFilter->Update();
                    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
                    polydata->ShallowCopy(vertexFilter->GetOutput());
                    polydata->GetPointData()->SetScalars(colors);

                    // Visualization
                    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                    mapper->SetInputData(polydata);

                    if(actor){
                        renderer->RemoveActor(actor);
                    }
                        
                    actor = vtkSmartPointer<vtkActor>::New();
                    actor->SetMapper(mapper);
                    actor->GetProperty()->SetPointSize(5);

                    renderer->AddActor(actor);
                }

                idle_ = true;
            }

        };

        setPolicy(new PolicyAny());
        iPolicy_->setupStream("cloud");
        iPolicy_->setupStream("dataframe");

    }
}
