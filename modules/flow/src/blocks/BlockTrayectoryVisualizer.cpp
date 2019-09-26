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



#include <mico/flow/blocks/BlockTrayectoryVisualizer.h>

#include <mico/flow/policies/policies.h>

#include <Eigen/Eigen>

namespace mico{

    BlockTrayectoryVisualizer::BlockTrayectoryVisualizer(){

        // Setup the visualization pipeline
        mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();

        actor_ = vtkSmartPointer<vtkActor>::New();
        actor_->SetMapper(mapper_);

        renderer_ = vtkSmartPointer<vtkRenderer>::New();
        renderer_->AddActor(actor_);

        window_ = vtkSmartPointer<vtkRenderWindow>::New();
        window_->AddRenderer(renderer_);

        interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor_->SetRenderWindow(window_);

        colors_ = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors_->SetNumberOfComponents(3);

        pts_ = vtkSmartPointer<vtkPoints>::New();
        lines_ = vtkSmartPointer<vtkCellArray>::New();

        linesPolyData_ = vtkSmartPointer<vtkPolyData>::New();
        linesPolyData_->GetCellData()->SetScalars(colors_);
        linesPolyData_->SetPoints(pts_);
        linesPolyData_->SetLines(lines_);

        mapper_->SetInputData(linesPolyData_);

        // Add origin
        double origin[3] = { 0.0, 0.0, 0.0 };
        pts_->InsertNextPoint(origin);

        
        // Visualize
        interactorThread_ = std::thread([&](){
            interactor_->Start();
        });

        callback_ = [&](std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid){
            if(idle_){
                idle_ = false;
                
                Eigen::Matrix4f pose = std::any_cast<Eigen::Matrix4f>(_data["pose"]);
                double pt[3] = {    (double) pose(0,3), 
                                    (double) pose(1,3), 
                                    (double) pose(2,3)};
                pts_->InsertNextPoint(pt);
                
                vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                line->GetPointIds()->SetId(0, currentIdx_); // the second 0 is the index of the Origin in linesPolyData's points
                line->GetPointIds()->SetId(1, currentIdx_++); // the second 1 is the index of P0 in linesPolyData's points

                lines_->InsertNextCell(line);
                colors_->InsertNextTupleValue(green);

                window_->Render();
                idle_ = true;
            }

        };

        setPolicy(new PolicyAllRequired()); // 666 OH SHIT THE ORDER IS IMPORTANT!
        iPolicy_->setupStream("pose");

    }
}
