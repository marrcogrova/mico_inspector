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



#include <mico/flow/blocks/visualizers/BlockTrayectoryVisualizer.h>

#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

#include <Eigen/Eigen>

#include <vtkInteractorStyleFlight.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>


namespace mico{

    BlockTrayectoryVisualizer::BlockTrayectoryVisualizer(){

        // Setup render window, renderer, and interactor
        renderWindow->SetWindowName("Trajectory Visualization");
        renderWindow->AddRenderer(renderer);
        renderWindowInteractor->SetRenderWindow(renderWindow);
        spinOnceCallback_ = vtkSmartPointer<SpinOnceCallback>::New();
        spinOnceCallback_->interactor_ = renderWindowInteractor;
        renderWindowInteractor->AddObserver(SpinOnceCallback::TimerEvent, spinOnceCallback_);
        renderer->SetBackground(colors->GetColor3d("Gray").GetData());

        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        widgetCoordinates_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        widgetCoordinates_->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
        widgetCoordinates_->SetOrientationMarker( axes );
        widgetCoordinates_->SetInteractor( renderWindowInteractor );
        widgetCoordinates_->SetViewport( 0.0, 0.0, 0.4, 0.4 );
        widgetCoordinates_->SetEnabled( 1 );
        // widgetCoordinates_->InteractiveOn();


        // Visualize
        runInteractor_ = true;
        interactorThread_ = std::thread([&](){
            renderWindowInteractor->Initialize();
            while(runInteractor_){
                renderWindowInteractor->Render();
                auto timerId = renderWindowInteractor->CreateRepeatingTimer (10);    
                renderWindowInteractor->Start();
                renderWindowInteractor->DestroyTimer(timerId);
            }
        });

        // Init trajectory
        points->InsertNextPoint(0, 0, 0);

        polyData->Allocate();
        polyData->SetPoints(points);
        mapper->SetInputData(polyData);
        actor->SetMapper(mapper);
        renderer->AddActor(actor);

        iPolicy_ = new Policy({"pose"});

        iPolicy_->setCallback({"pose"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        Eigen::Matrix4f pose = std::any_cast<Eigen::Matrix4f>(_data["pose"]);

                                        vtkIdType connectivity[2];
                                        connectivity[0] = currentIdx_;
                                        connectivity[1] = currentIdx_+1;
                                        points->InsertNextPoint(pose(0,3), pose(1,3), pose(2,3));
                                        polyData->InsertNextCell(VTK_LINE,2,connectivity);
                                        currentIdx_++;

                                        polyData->Modified();
                                        
                                        idle_ = true;
                                    }

                                }
                            );
    }

    BlockTrayectoryVisualizer::~BlockTrayectoryVisualizer(){
        runInteractor_ = false;
        if(interactorThread_.joinable()){
            interactorThread_.join();
        }
        renderWindowInteractor->GetRenderWindow()->Finalize();
        renderWindowInteractor->ExitCallback();
        renderWindowInteractor->TerminateApp();
    }

}
